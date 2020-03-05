#include "drive/driver.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include <Eigen/Dense>
#include <vector>

#include "drive/config.h"
#include "drive/controller.h"
#include "hw/cam/cam.h"
#include "hw/car/car.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "io/flushthread.h"
#include "localization/ceiltrack/ceiltrack.h"
#include "ui/display.h"

// hardcoded garbage for the time being
const float CEILHOME_X = -3.03, CEILHOME_Y = 0.73, CEILHOME_THETA = 0;
const float CEIL_HEIGHT = 8.25*0.3048;
const float CEIL_X_GRID = 0.3048*10/CEIL_HEIGHT;
const float CEIL_Y_GRID = 0.3048*12/CEIL_HEIGHT;

// finish line for built-in lap timer
const float FINISHX = 9.5;
const float FINISHY = 160/60.0;

// const int PWMCHAN_STEERING = 14;
// const int PWMCHAN_ESC = 15;

struct CarState {
  Eigen::Vector3f accel, gyro;
  int8_t throttle, steering;
  float wheel_dist, wheel_v;
  float ceiltrack_pos[3];

  CarState() : accel(0, 0, 0), gyro(0, 0, 0) {
    throttle = 0;
    steering = 0;
    wheel_dist = 0;
    wheel_v = 0;
    SetHome();
  }

  void SetHome() {
    ceiltrack_pos[0] = CEILHOME_X;
    ceiltrack_pos[1] = CEILHOME_Y;
    ceiltrack_pos[2] = CEILHOME_THETA;
  }

  // 2 3-float vectors, 3 uint8s, 2 4-uint16 arrays
  int SerializedSize() { return 8 + 4 * 3 * 2 + 2 + 2 * 4; }

  int Serialize(uint8_t *buf, int bufsiz) {
    int len = SerializedSize();
    assert(bufsiz >= len);
    memcpy(buf, "CSt1", 4);
    memcpy(buf + 4, &len, 4);
    buf += 8;
    memcpy(buf + 0, &throttle, 1);
    memcpy(buf + 1, &steering, 1);
    memcpy(buf + 2, &accel[0], 4);
    memcpy(buf + 2 + 4, &accel[1], 4);
    memcpy(buf + 2 + 8, &accel[2], 4);
    memcpy(buf + 14, &gyro[0], 4);
    memcpy(buf + 14 + 4, &gyro[1], 4);
    memcpy(buf + 14 + 8, &gyro[2], 4);
    memcpy(buf + 26, &wheel_dist, 4);
    memcpy(buf + 30, &wheel_v, 4);
    return len;
  }
} carstate_;

Driver::Driver(IMU *imu, JoystickInput *js, UIDisplay *disp)
    : imu_(imu),
      js_(js),
      display_(disp),
      gyro_last_(0, 0, 0),
      gyro_bias_(0, 0, 0),
      accel_last_(0, 0, 0),
      accel_bias_(0, 0, 0) {
  log_fd_ = -1;
  h264_fd_ = -1;
  log_frame_ = 0;
  autodrive_ = false;
  memset(&last_t_, 0, sizeof(last_t_));
  memset(&last_lap_, 0, sizeof(last_lap_));
  js_throttle_ = 0;
  js_steering_ = 0;

  config_item_ = 0;
  x_down_ = y_down_ = false;
  done_ = false;
}

bool Driver::Init(const INIReader &ini) {
  float fx, fy, cx, cy, k1;
  std::string camcal = ini.GetString("camera", "calibration", "");
  if (camcal == "" || sscanf(camcal.c_str(), "%f %f %f %f %f", &fx, &fy, &cx, &cy, &k1) != 5) {
    fprintf(stderr, "missing or invalid [camera].calibration in .ini file!\n");
    fprintf(stderr, "syntax:\n[camera]\ncalibration=fx fy cx cy k1\n");
    return false;
  }
  // adjust for 640x480
  lens_.SetCalibration(fx/4.05, fy/4.05, cx/4.05, cy/4.05, k1);
  float camrot = ini.GetReal("camera", "rotation", 22) * M_PI / 180.0;

  if (!ceiltrack_.Init(lens_, camrot)) {
    fprintf(stderr, "ceiltrack init failure");
    return false;
  }

  if (!flush_thread_.Init()) {
    return false;
  }

  if (display_) {
    display_->InitCamera(lens_, camrot);
  }

  if (config_.Load()) {
    fprintf(stderr, "Loaded driver configuration\n");
  }

  // FIXME(a1k0n): use lens calibration, not floorlut.bin
  // but we still need the mask defined somehow
  if (!obstacledetect_.Open("floorlut.bin")) {
    fprintf(stderr, "can't open floorlut.bin, obstacle detection lookup table");
    return false;
  }

  return true;
}

bool Driver::StartRecording(const char *logname, const char *h264name) {
  log_frame_ = 0;
  log_fd_ = open(logname, O_CREAT | O_TRUNC | O_WRONLY, 0666);
  if (log_fd_ == -1) {
    perror(logname);
    return false;
  }
  h264_fd_ = open(h264name, O_CREAT | O_TRUNC | O_WRONLY, 0666);
  if (h264_fd_ == -1) {
    perror(h264name);
    close(log_fd_);
    log_fd_ = -1;
    return false;
  }
  printf("--- recording %s ---\n", logname);
  // write header IFF chunk immediately: store the car config
  int siz = config_.SerializedSize();
  uint8_t *hdrbuf = new uint8_t[siz];
  config_.Serialize(hdrbuf, siz);
  write(log_fd_, hdrbuf, siz);
  delete[] hdrbuf;
  return true;
}

bool Driver::IsRecording() { return log_fd_ != -1; }

void Driver::StopRecording() {
  if (log_fd_ == -1) {
    return;
  }
  flush_thread_.AddEntry(log_fd_, NULL, -1);
  flush_thread_.AddEntry(h264_fd_, NULL, -1);
  log_fd_ = -1;
  h264_fd_ = -1;
}

Driver::~Driver() { StopRecording(); }

// recording data is in IFF format, can be read with python chunk interface:
// ck = chunk.Chunk(file, align=False, bigendian=False, inclheader=True)
// each frame is stored in a CYCF chunk which includes an 8-byte timestamp,
// and further set of chunks encoded by each piece below.
void Driver::QueueRecordingData(const timeval &t) {
  uint32_t chunklen = 8 + 8;           // iff header, timestamp
  uint32_t camframelen = 4 + 8;  // iff header, width, camera frame
  // each of the following entries is expected to be a valid
  // IFF chunk on its own
  chunklen += carstate_.SerializedSize();
  chunklen += controller_.SerializedSize();
  chunklen += camframelen;

  // copy our frame, push it onto a stack to be flushed
  // asynchronously to sdcard
  uint8_t *chunkbuf = new uint8_t[chunklen];
  // write length + timestamp header
  memcpy(chunkbuf, "CYCF", 4);
  memcpy(chunkbuf + 4, &chunklen, 4);
  memcpy(chunkbuf + 8, &t.tv_sec, 4);
  memcpy(chunkbuf + 12, &t.tv_usec, 4);
  int ptr = 16;
  ptr += carstate_.Serialize(chunkbuf + ptr, chunklen - ptr);
  ptr += controller_.Serialize(chunkbuf + ptr, chunklen - ptr);

  // write the 640x480 yuv420 buffer last
  memcpy(chunkbuf + ptr, "vfra", 4);
  memcpy(chunkbuf + ptr + 4, &camframelen, 4);
  memcpy(chunkbuf + ptr + 8, &log_frame_, 4);

  flush_thread_.AddEntry(log_fd_, chunkbuf, chunklen);
}

  // Update controller from gyro and wheel encoder inputs

  // Update controller and UI from camera
void Driver::UpdateFromCamera(uint8_t *buf, float dt) {
  float prevxy[2];
  prevxy[0] = -carstate_.ceiltrack_pos[0] * CEIL_HEIGHT;
  prevxy[1] = -carstate_.ceiltrack_pos[1] * CEIL_HEIGHT;

  ceiltrack_.Update(buf, 240, CEIL_X_GRID, CEIL_Y_GRID, carstate_.ceiltrack_pos,
                    2, false);
  float xytheta[3];
  // convert ceiling homogeneous coordinates to actual meters on the ground
  // also we need to convert from bottom-up to top-down coordinates so we negate
  // through
  xytheta[0] = -carstate_.ceiltrack_pos[0] * CEIL_HEIGHT;
  xytheta[1] = -carstate_.ceiltrack_pos[1] * CEIL_HEIGHT;
  xytheta[2] = -carstate_.ceiltrack_pos[2];

  // lap timer
  if (prevxy[0] < FINISHX && xytheta[0] >= FINISHX && xytheta[1] < FINISHY) {
    if (last_lap_.tv_sec != 0) {
      float laptime = (last_t_.tv_sec - last_lap_.tv_sec) +
                      (last_t_.tv_usec - last_lap_.tv_usec) * 1e-6;
      printf("### lap time %0.3f ", laptime);
      // dump configuration
      uint16_t *dc = reinterpret_cast<uint16_t*>(&config_);
      for (int i = 0; i < config_.N_CONFIGITEMS; i++) {
        printf("%d ", dc[i]);
      }
      printf("\n");
    } else {
      fprintf(stderr, "Starting first lap...\n");
    }
    last_lap_ = last_t_;
  }

  obstacledetect_.Update(buf, 40, 150);  // FIXME(a1k0n): needs config
  const int32_t *pcar = obstacledetect_.GetCarPenalties();
  const int32_t *pcone = obstacledetect_.GetConePenalties();

  controller_.UpdateLocation(config_, xytheta);
  controller_.Plan(config_, pcar, pcone);

  // display_.UpdateConeView(buf, 0, NULL);
  // display_->UpdateEncoders(carstate_.wheel_pos);
  // FIXME: hardcoded map size 20mx10m
  if (display_) {
    static std::vector<std::pair<float, float>> gridpts;
    gridpts.clear();
    ceiltrack_.GetMatchedGrid(lens_, carstate_.ceiltrack_pos, CEIL_X_GRID, CEIL_Y_GRID, &gridpts);
    display_->UpdateCameraView(buf, gridpts);
    display_->UpdateCeiltrackView(xytheta, CEIL_X_GRID * CEIL_HEIGHT,
                                  CEIL_Y_GRID * CEIL_HEIGHT, 20, 10,
                                  pcar, pcone, carstate_.wheel_v);
  }
}

  // Called each camera frame, 30Hz
void Driver::OnCameraFrame(uint8_t *buf, size_t length) {
  struct timeval t;
  gettimeofday(&t, NULL);

  float dt = t.tv_sec - last_t_.tv_sec + (t.tv_usec - last_t_.tv_usec) * 1e-6;
  if (dt > 0.1 && last_t_.tv_sec != 0) {
    fprintf(stderr,
            "CameraThread::OnFrame: WARNING: "
            "%fs gap between frames?!\n",
            dt);
  }
  last_t_ = t;

  UpdateFromCamera(buf, dt);

  if (IsRecording()) {
    Camera::EncodeFrame(buf, length, log_frame_ == 0);
    QueueRecordingData(t);
    log_frame_++;
  }
}

void Driver::OnH264Frame(uint8_t *buf, size_t length) {
  if (h264_fd_ != -1) {
    // make a memcpy for output buffering
    uint8_t *fbuf = new uint8_t[length];
    memcpy(fbuf, buf, length);
    flush_thread_.AddEntry(h264_fd_, fbuf, length);
  }
}

// Called each control loop frame, 100Hz
// N.B. this can be called concurrently with OnFrame in a separate thread
bool Driver::OnControlFrame(CarHW *car, float dt) {
  if (js_) {
    js_->ReadInput(this);
  }

  Eigen::Vector3f gyro, accel;
  imu_->ReadIMU(&accel, &gyro);
  gyro_last_ = 0.95 * gyro_last_ + 0.05 * gyro;
  accel_last_ = 0.95 * accel_last_ + 0.05 * accel;
  carstate_.gyro = gyro - gyro_bias_;
  carstate_.accel = accel - accel_bias_;

  // a = v^2 k = v w
  // v = a/w
  float ds, v;
  if (car->GetWheelMotion(&ds, &v)) {  // use wheel encoders if we have 'em
    carstate_.wheel_v = v;
  } else {
    // otherwise try to use the acceleromters/gyros to guess
    // FIXME(a1k0n): do these axes need configuration in the .ini?
    carstate_.wheel_v = 0.95 * (carstate_.wheel_v - 9.8*carstate_.accel[1]*dt);
    if (fabsf(carstate_.gyro[2]) > 0.1) {
      carstate_.wheel_v +=
          0.05 * fabsf(9.8 * carstate_.accel[0] / carstate_.gyro[2]);
    }
    if (carstate_.wheel_v < 0) {
      carstate_.wheel_v = 0;
    }
  }
  controller_.UpdateState(config_, carstate_.accel, carstate_.gyro,
                          carstate_.wheel_v, dt);

  float u_a = carstate_.throttle / 127.0;
  float u_s = carstate_.steering / 127.0;
  if (controller_.GetControl(config_, js_throttle_ / 32767.0,
                             js_steering_ / 32767.0, &u_a, &u_s, dt,
                             autodrive_)) {
    uint8_t leds = (log_frame_ & 4);    // blink green LED
    car->SetControls(leds, u_a, u_s);
  }
  carstate_.throttle = 127*u_a;
  carstate_.steering = 127*u_s;

  return !done_;
}

void Driver::OnDPadPress(char direction) {
  int16_t *value = ((int16_t *)&config_) + config_item_;
  switch (direction) {
    case 'U':
      --config_item_;
      if (config_item_ < 0) config_item_ = DriverConfig::N_CONFIGITEMS - 1;
      fprintf(stderr, "\n");
      break;
    case 'D':
      ++config_item_;
      if (config_item_ >= DriverConfig::N_CONFIGITEMS) config_item_ = 0;
      fprintf(stderr, "\n");
      break;
    case 'L':
      if (y_down_) {
        *value -= 100;
      } else if (x_down_) {
        *value -= 10;
      } else {
        --*value;
      }
      break;
    case 'R':
      if (y_down_) {
        *value += 100;
      } else if (x_down_) {
        *value += 10;
      } else {
        ++*value;
      }
      break;
  }
  UpdateDisplay();
}

void Driver::OnButtonPress(char button) {
  struct timeval tv;
  gettimeofday(&tv, NULL);

  switch (button) {
    case '+':  // start button: start recording
      if (!IsRecording()) {
        char fnamebuf[256], fnamebuf2[256];
        time_t start_time = time(NULL);
        struct tm start_time_tm;
        localtime_r(&start_time, &start_time_tm);
        strftime(fnamebuf, sizeof(fnamebuf), "cycloid-%Y%m%d-%H%M%S.rec",
                 &start_time_tm);
        strftime(fnamebuf2, sizeof(fnamebuf2), "cycloid-%Y%m%d-%H%M%S.h264",
                 &start_time_tm);
        if (StartRecording(fnamebuf, fnamebuf2)) {
          fprintf(stderr, "%ld.%06ld started recording %s\n", tv.tv_sec,
                  tv.tv_usec, fnamebuf);
          if (display_) display_->UpdateStatus(fnamebuf, 0xffe0);
        }
      }
      break;
    case '-':  // select button: stop recording
      if (IsRecording()) {
        StopRecording();
        fprintf(stderr, "%ld.%06ld stopped recording\n", tv.tv_sec, tv.tv_usec);
        if (display_) display_->UpdateStatus("recording stopped", 0xffff);
      }
      break;
    case 'H':  // home button: init to start line
      carstate_.SetHome();
      gyro_bias_ = gyro_last_;
      accel_bias_ = accel_last_;
      printf("gyro bias %0.3f %0.3f %0.3f\n", gyro_bias_[0], gyro_bias_[1],
             gyro_bias_[2]);
      printf("accel bias %0.3f %0.3f %0.3f\n", accel_bias_[0], accel_bias_[1],
             accel_bias_[2]);
      if (display_) display_->UpdateStatus("starting line", 0x07e0);
      break;
    case 'L':
      if (!autodrive_) {
        fprintf(stderr, "%ld.%06ld autodrive ON\n", tv.tv_sec, tv.tv_usec);
        autodrive_ = true;
      }
      break;
    case 'R':
      if (display_) {
        display_->NextMode();
      }
      break;
    case 'B':
      controller_.ResetState();
      if (config_.Load()) {
        fprintf(stderr, "config loaded\n");
        int16_t *values = ((int16_t *)&config_);
        if (display_) {
          display_->UpdateConfig(DriverConfig::confignames,
                                 DriverConfig::N_CONFIGITEMS, config_item_,
                                 values);
          display_->UpdateStatus("config loaded", 0xffff);
        }
      }
      fprintf(stderr, "reset kalman filter\n");
      break;
    case 'A':
      if (config_.Save()) {
        fprintf(stderr, "config saved\n");
        if (display_) display_->UpdateStatus("config saved", 0xffff);
      }
      break;
    case 'X':
      x_down_ = true;
      break;
    case 'Y':
      y_down_ = true;
      break;
  }
}

void Driver::OnButtonRelease(char button) {
  struct timeval tv;
  gettimeofday(&tv, NULL);

  switch (button) {
    case 'L':
      if (autodrive_) {
        autodrive_ = false;
        fprintf(stderr, "%ld.%06ld autodrive OFF\n", tv.tv_sec, tv.tv_usec);
      }
      break;
    case 'X':
      x_down_ = false;
      break;
    case 'Y':
      y_down_ = false;
      break;
  }
}

void Driver::OnAxisMove(int axis, int16_t value) {
  switch (axis) {
    case 1:  // left stick y axis
      js_throttle_ = -value;
      break;
    case 2:  // right stick x axis
      js_steering_ = value;
      break;
  }
}

void Driver::UpdateDisplay() {
  // hack because all config values are int16_t's in 1/100th steps
  int16_t *values = ((int16_t *)&config_);
  int16_t value = values[config_item_];
  // FIXME: does this work for negative values?
  fprintf(stderr, "%s %d.%02d\r", DriverConfig::confignames[config_item_], value / 100,
          value % 100);

  if (display_)
    display_->UpdateConfig(DriverConfig::confignames,
                           DriverConfig::N_CONFIGITEMS, config_item_, values);
}

