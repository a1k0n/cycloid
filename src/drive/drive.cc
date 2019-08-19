#include <fcntl.h>
#include <fenv.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "drive/config.h"
#include "drive/controller.h"
#include "drive/flushthread.h"
#include "hw/cam/cam.h"
#include "hw/car/stm32rs232.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "localization/ceiltrack/ceiltrack.h"
#include "ui/display.h"

volatile bool done = false;

// hardcoded garbage for the time being
const float CEILHOME_X = -3.03, CEILHOME_Y = 0.73, CEILHOME_THETA = 0;
const float CEIL_HEIGHT = 8.25*0.3048;
const float CEIL_X_GRID = 0.3048*10/CEIL_HEIGHT;
const float CEIL_Y_GRID = 0.3048*12/CEIL_HEIGHT;


// const int PWMCHAN_STEERING = 14;
// const int PWMCHAN_ESC = 15;

void handle_sigint(int signo) { done = true; }

I2C i2c;
STM32HatSerial stm32hat;
IMU *imu_;
UIDisplay display_;
FlushThread flush_thread_;
CeilingTracker ceiltrack_;
int16_t js_throttle_ = 0, js_steering_ = 0;

struct CarState {
  Eigen::Vector3f accel, gyro;
  uint8_t servo_pos;
  uint16_t wheel_pos[4];
  uint16_t wheel_dt[4];
  int8_t throttle, steering;
  float ceiltrack_pos[3];

  CarState(): accel(0, 0, 0), gyro(0, 0, 0) {
    memset(wheel_pos, 0, 8);
    memset(wheel_dt, 0, 8);
    servo_pos = 110;
    throttle = 0;
    steering = 0;
    SetHome();
  }

  void SetHome() {
    ceiltrack_pos[0] = CEILHOME_X;
    ceiltrack_pos[1] = CEILHOME_Y;
    ceiltrack_pos[2] = CEILHOME_THETA;
  }

  // 2 3-float vectors, 3 uint8s, 2 4-uint16 arrays
  int SerializedSize() { return 8 + 4 * 3 * 2 + 3 + 2 * 4 * 2; }

  int Serialize(uint8_t *buf, int bufsiz) {
    int len = SerializedSize();
    assert(bufsiz >= len);
    memcpy(buf, "CSta", 4);
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
    memcpy(buf + 26, &servo_pos, 1);
    memcpy(buf + 27, wheel_pos, 2 * 4);
    memcpy(buf + 35, wheel_dt, 2 * 4);
    return len;
  }
} carstate_;

class Driver: public CameraReceiver {
 public:
  explicit Driver() {
    output_fd_ = -1;
    frame_ = 0;
    frameskip_ = 0;
    autodrive_ = false;
    memset(&last_t_, 0, sizeof(last_t_));
    if (config_.Load()) {
      fprintf(stderr, "Loaded driver configuration\n");
    }
    firstframe_ = true;
    ncones_ = 0;
  }

  bool StartRecording(const char *fname, int frameskip) {
    frameskip_ = frameskip;
    frame_ = 0;
    if (!strcmp(fname, "-")) {
      output_fd_ = fileno(stdout);
    } else {
      output_fd_ = open(fname, O_CREAT|O_TRUNC|O_WRONLY, 0666);
    }
    if (output_fd_ == -1) {
      perror(fname);
      return false;
    }
    printf("--- recording %s ---\n", fname);
    // write header IFF chunk immediately: store the car config
    int siz = config_.SerializedSize();
    uint8_t *hdrbuf = new uint8_t[siz];
    config_.Serialize(hdrbuf, siz);
    write(output_fd_, hdrbuf, siz);
    delete[] hdrbuf;
    return true;
  }

  bool IsRecording() {
    return output_fd_ != -1;
  }

  void StopRecording() {
    if (output_fd_ == -1) {
      return;
    }
    flush_thread_.AddEntry(output_fd_, NULL, -1);
    output_fd_ = -1;
  }

  ~Driver() {
    StopRecording();
  }

  // recording data is in IFF format, can be read with python chunk interface:
  // ck = chunk.Chunk(file, align=False, bigendian=False, inclheader=True)
  // each frame is stored in a CYCF chunk which includes an 8-byte timestamp,
  // and further set of chunks encoded by each piece below.
  void QueueRecordingData(const timeval &t, uint8_t *buf, size_t length) {
    uint32_t chunklen = 8 + 8;  // iff header, timestamp
    uint32_t yuvcklen = length + 8 + 2;  // iff header, width, camera frame
    // each of the following entries is expected to be a valid
    // IFF chunk on its own
    chunklen += carstate_.SerializedSize();
    chunklen += controller_.SerializedSize();
    chunklen += yuvcklen;

    // copy our frame, push it onto a stack to be flushed
    // asynchronously to sdcard
    uint8_t *chunkbuf = new uint8_t[chunklen];
    // write length + timestamp header
    memcpy(chunkbuf, "CYCF", 4);
    memcpy(chunkbuf+4, &chunklen, 4);
    memcpy(chunkbuf+8, &t.tv_sec, 4);
    memcpy(chunkbuf+12, &t.tv_usec, 4);
    int ptr = 16;
    ptr += carstate_.Serialize(chunkbuf+ptr, chunklen - ptr);
    ptr += controller_.Serialize(chunkbuf+ptr, chunklen - ptr);

    // write the 640x480 yuv420 buffer last
    memcpy(chunkbuf+ptr, "Y420", 4);
    memcpy(chunkbuf+ptr+4, &yuvcklen, 4);
    uint16_t framewidth = 640;  // hardcoded, fixme
    memcpy(chunkbuf+ptr+8, &framewidth, 2);
    memcpy(chunkbuf+ptr+10, buf, length);

    flush_thread_.AddEntry(output_fd_, chunkbuf, chunklen);
  }

  // Update controller from gyro and wheel encoder inputs

  // Update controller and UI from camera
  void UpdateFromCamera(uint8_t *buf, float dt) {
    if (firstframe_) {
      memcpy(last_encoders_, carstate_.wheel_pos, 4*sizeof(uint16_t));
      firstframe_ = false;
      dt = 1.0 / 30.0;
    }

    ceiltrack_.Update(buf, 240, CEIL_X_GRID, CEIL_Y_GRID, carstate_.ceiltrack_pos, 2, false);
    float xytheta[3];
    // convert ceiling homogeneous coordinates to actual meters on the ground
    // also we need to convert from bottom-up to top-down coordinates so we negate through
    xytheta[0] = -carstate_.ceiltrack_pos[0] * CEIL_HEIGHT;
    xytheta[1] = -carstate_.ceiltrack_pos[1] * CEIL_HEIGHT;
    xytheta[2] = -carstate_.ceiltrack_pos[2];
    controller_.UpdateLocation(config_, xytheta);
    controller_.Plan(config_);

    // display_.UpdateConeView(buf, 0, NULL);
    display_.UpdateEncoders(carstate_.wheel_pos);
    // FIXME: hardcoded map size 20mx10m
    display_.UpdateCeiltrackView(xytheta, CEIL_X_GRID*CEIL_HEIGHT, CEIL_Y_GRID*CEIL_HEIGHT, 20, 10);
    memcpy(last_encoders_, carstate_.wheel_pos, 4 * sizeof(uint16_t));
  }

  // Called each camera frame, 30Hz
  void OnFrame(uint8_t *buf, size_t length) {
    struct timeval t;
    gettimeofday(&t, NULL);
    frame_++;

    float dt = t.tv_sec - last_t_.tv_sec + (t.tv_usec - last_t_.tv_usec) * 1e-6;
    if (dt > 0.1 && last_t_.tv_sec != 0) {
      fprintf(stderr, "CameraThread::OnFrame: WARNING: "
          "%fs gap between frames?!\n", dt);
    }

    UpdateFromCamera(buf, dt);

    if (IsRecording() && frame_ > frameskip_) {
      frame_ = 0;
      QueueRecordingData(t, buf, length);
    }

    last_t_ = t;
  }

  // Called each control loop frame, 100Hz
  // N.B. this can be called concurrently with OnFrame in a separate thread
  void OnControlFrame(float ds, float dt) {
    controller_.UpdateState(config_,
        carstate_.accel, carstate_.gyro,
        carstate_.servo_pos, carstate_.wheel_dt, dt);

    float u_a = carstate_.throttle / 127.0;
    float u_s = carstate_.steering / 127.0;
    if (controller_.GetControl(config_, js_throttle_ / 32767.0,
          js_steering_ / 32767.0, &u_a, &u_s, dt, autodrive_, frame_)) {
      carstate_.steering = 127 * u_s;
      carstate_.throttle = 127 * u_a;

      // override steering for servo angle calibration
      if (calibration_ == CAL_SRV_RIGHT) {
        carstate_.steering = -64;
      } else if (calibration_ == CAL_SRV_LEFT) {
        carstate_.steering = 64;
      }

      uint8_t leds = (frame_ & 4);  // blink green LED
      leds |= IsRecording() ? 2 : 0;  // solid red when recording
      stm32hat.SetControls(leds, carstate_.throttle, carstate_.steering);
    }
  }

  bool autodrive_;
  DriveController controller_;
  DriverConfig config_;
  int frame_;

  bool firstframe_;
  uint16_t last_encoders_[4];

  int ncones_;
  int conesx_[16];
  float conestheta_[16];

  enum {
    CAL_NONE,
    CAL_SRV_LEFT,
    CAL_SRV_RIGHT,
  } calibration_;

 private:
  int output_fd_;
  int frameskip_;
  struct timeval last_t_;
};

Driver driver_;

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

class DriverInputReceiver : public InputReceiver {
 private:
  static const char *configmenu[];  // initialized below
  static const int N_CONFIGITEMS;

  int config_item_;
  DriverConfig *config_;
  bool x_down_, y_down_;

 public:
  explicit DriverInputReceiver(DriverConfig *config) {
    config_ = config;
    config_item_ = 0;
    x_down_ = y_down_ = false;
    UpdateDisplay();
  }

  virtual void OnDPadPress(char direction) {
    int16_t *value = ((int16_t*) config_) + config_item_;
    switch (direction) {
      case 'U':
        --config_item_;
        if (config_item_ < 0)
          config_item_ = N_CONFIGITEMS - 1;
        fprintf(stderr, "\n");
        break;
      case 'D':
        ++config_item_;
        if (config_item_ >= N_CONFIGITEMS)
          config_item_ = 0;
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

  virtual void OnButtonPress(char button) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    switch (button) {
      case '+':  // start button: start recording
        if (!driver_.IsRecording()) {
          char fnamebuf[256];
          time_t start_time = time(NULL);
          struct tm start_time_tm;
          localtime_r(&start_time, &start_time_tm);
          strftime(fnamebuf, sizeof(fnamebuf), "cycloid-%Y%m%d-%H%M%S.rec",
              &start_time_tm);
          if (driver_.StartRecording(fnamebuf, 0)) {
            fprintf(stderr, "%ld.%06ld started recording %s\n",
                tv.tv_sec, tv.tv_usec, fnamebuf);
            display_.UpdateStatus(fnamebuf, 0xffe0);
          }
        }
        break;
      case '-':  // select button: stop recording
        if (driver_.IsRecording()) {
          driver_.StopRecording();
          fprintf(stderr, "%ld.%06ld stopped recording\n",
              tv.tv_sec, tv.tv_usec);
          display_.UpdateStatus("recording stopped", 0xffff);
        }
        break;
      case 'H':  // home button: init to start line
        carstate_.SetHome();
        display_.UpdateStatus("starting line", 0x07e0);
        break;
      case 'L':
         if (!driver_.autodrive_) {
          fprintf(stderr, "%ld.%06ld autodrive ON\n", tv.tv_sec, tv.tv_usec);
          driver_.autodrive_ = true;
        }
        break;
      case 'R':
        driver_.calibration_ = Driver::CAL_SRV_RIGHT;
        break;
      case 'B':
        driver_.controller_.ResetState();
        if (config_->Load()) {
          fprintf(stderr, "config loaded\n");
          int16_t *values = ((int16_t*) config_);
          display_.UpdateConfig(configmenu, N_CONFIGITEMS,
              config_item_, values);
          display_.UpdateStatus("config loaded", 0xffff);
        }
        fprintf(stderr, "reset kalman filter\n");
        break;
      case 'A':
        if (config_->Save()) {
          fprintf(stderr, "config saved\n");
          display_.UpdateStatus("config saved", 0xffff);
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

  virtual void OnButtonRelease(char button) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    switch (button) {
      case 'L':
        if (driver_.autodrive_) {
          driver_.autodrive_ = false;
          fprintf(stderr, "%ld.%06ld autodrive OFF\n", tv.tv_sec, tv.tv_usec);
        }
        // fall through
      case 'R':
        driver_.calibration_ = Driver::CAL_NONE;
        break;
      case 'X':
        x_down_ = false;
        break;
      case 'Y':
        y_down_ = false;
        break;
    }
  }

  virtual void OnAxisMove(int axis, int16_t value) {
    switch (axis) {
      case 1:  // left stick y axis
        js_throttle_ = -value;
        break;
      case 3:  // right stick x axis
        // changed! 2 for Wii U Pro controller, 3 for Logitech F710
        js_steering_ = value;
        break;
    }
  }

  void UpdateDisplay() {
    // hack because all config values are int16_t's in 1/100th steps
    int16_t *values = ((int16_t*) config_);
    int16_t value = values[config_item_];
    // FIXME: does this work for negative values?
    fprintf(stderr, "%s %d.%02d\r", configmenu[config_item_],
        value / 100, value % 100);
    display_.UpdateConfig(configmenu, N_CONFIGITEMS, config_item_, values);
  }
};

const char *DriverInputReceiver::configmenu[] = {
  "max speed",
  "traction limit",
  "lookahead dist",
  "lookahead dkdt",
  "cone/lane penalty",
  "motorcontrol gain",
  "motorcontrol kI",
  "turn-in lift",
  "servo rate",
  "servo offset",
  "servo finetune",
  "cone precision",
};
const int DriverInputReceiver::N_CONFIGITEMS =
    sizeof(configmenu) / sizeof(configmenu[0]);

int main(int argc, char *argv[]) {
  signal(SIGINT, handle_sigint);

  feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

  INIReader ini("cycloid.ini");
  {
    int inierr = ini.ParseError();
    if (inierr != 0) {
      if (inierr == -1) {
        fprintf(stderr, "please create cycloid.ini!\n");
      } else {
        fprintf(stderr, "error loading cycloid.ini on line %d\n", inierr);
      }
      return 1;
    }
  }

  int fps = ini.GetInteger("camera", "fps", 30);

  if (!flush_thread_.Init()) {
    return 1;
  }

  if (!Camera::Init(640, 480, fps))
    return 1;

  JoystickInput js;

  if (!i2c.Open()) {
    fprintf(stderr, "need to enable i2c in raspi-config, probably\n");
    return 1;
  }

  if (!display_.Init()) {
    fprintf(stderr, "run this:\n"
       "sudo modprobe fbtft_device name=adafruit22a rotate=90\n");
    // TODO(asloane): support headless mode
    return 1;
  }

  if (!ceiltrack_.Open("ceillut.bin")) {
    fprintf(stderr, "can't open ceillut.bin, camera calibration lookup table\n");
    return 1;
  }

  bool has_joystick = false;
  if (js.Open(ini)) {
    has_joystick = true;
  } else {
    fprintf(stderr, "joystick not detected, but continuing anyway!\n");
  }

  imu_ = IMU::GetI2CIMU(i2c, ini);
  if (!imu_ || !imu_->Init()) {
    fprintf(stderr, "unable to connect to IMU; aborting\n");
    return 1;
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  fprintf(stderr, "%ld.%06ld camera on @%d fps\n", tv.tv_sec, tv.tv_usec, fps);

  DriverInputReceiver input_receiver(&driver_.config_);
  if (!Camera::StartRecord(&driver_)) {
    return 1;
  }

  gettimeofday(&tv, NULL);
  fprintf(stderr, "%ld.%06ld started camera\n", tv.tv_sec, tv.tv_usec);

  uint16_t last_wpos = 0;
  stm32hat.Init();
  stm32hat.AwaitSync(&last_wpos, carstate_.wheel_dt);
  timeval last_t;
  gettimeofday(&last_t, NULL);

  while (!done) {
    if (!stm32hat.AwaitSync(carstate_.wheel_pos, carstate_.wheel_dt)) {
      continue;
    }

    if (has_joystick) {
      js.ReadInput(&input_receiver);
    }

    imu_->ReadIMU(&carstate_.accel, &carstate_.gyro);

    timeval t;
    gettimeofday(&t, NULL);

    // predict using front wheel distance
    uint16_t wheel_delta = carstate_.wheel_pos[0] - last_wpos;
    float ds = V_SCALE * wheel_delta;
    float dt = t.tv_sec - last_t.tv_sec + (t.tv_usec - last_t.tv_usec) * 1e-6;
    last_t = t;
    last_wpos = carstate_.wheel_pos[0];

    driver_.OnControlFrame(ds, dt);
    // timeval t1;
    // gettimeofday(&t1, NULL);
    // printf("%d.%06d %f\n", t1.tv_sec - t.tv_sec, t1.tv_usec - t.tv_usec, dt);
  }

  Camera::StopRecord();
}
