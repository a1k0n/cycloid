#include <fcntl.h>
#include <fenv.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "drive/config.h"
#include "drive/controller.h"
#include "drive/flushthread.h"
#include "drive/imgproc.h"
#include "hw/cam/cam.h"
// #include "hw/car/pca9685.h"
#include "hw/car/teensy.h"
#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "ui/display.h"

volatile bool done = false;

// ugh ugh ugh
int8_t throttle_ = 0, steering_ = 0;
int16_t js_throttle_ = 0, js_steering_ = 0;

// const int PWMCHAN_STEERING = 14;
// const int PWMCHAN_ESC = 15;

void handle_sigint(int signo) { done = true; }

I2C i2c;
// PCA9685 pca(i2c);
Teensy teensy(i2c);
IMU imu(i2c);
UIDisplay display_;
FlushThread flush_thread_;
Eigen::Vector3f accel_(0, 0, 0), gyro_(0, 0, 0);
uint8_t servo_pos_ = 110;
uint16_t wheel_pos_[4] = {0, 0, 0, 0};
uint16_t wheel_dt_[4] = {0, 0, 0, 0};

class Driver: public CameraReceiver {
 public:
  Driver() {
    output_fd_ = -1;
    frame_ = 0;
    frameskip_ = 0;
    autosteer_ = false;
    gettimeofday(&last_t_, NULL);
    if (config_.Load()) {
      fprintf(stderr, "Loaded driver configuration\n");
    }
  }

  bool StartRecording(const char *fname, int frameskip) {
    frameskip_ = frameskip;
    if (!strcmp(fname, "-")) {
      output_fd_ = fileno(stdout);
    } else {
      output_fd_ = open(fname, O_CREAT|O_TRUNC|O_WRONLY, 0666);
    }
    if (output_fd_ == -1) {
      perror(fname);
      return false;
    }
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

  void OnFrame(uint8_t *buf, size_t length) {
    struct timeval t;
    gettimeofday(&t, NULL);
    frame_++;
    int32_t *reprojected = imgproc::Reproject(buf);
    // convert 32-bit buf down to 8 bits for recording
    const uint32_t imgsiz = imgproc::uxsiz * imgproc::uysiz * 3;
    uint8_t topview[imgsiz];
    for (int i = 0; i < imgsiz; i++) {
      topview[i] = reprojected[i];
    }

    if (IsRecording() && frame_ > frameskip_) {
      frame_ = 0;
      // save reprojected low-res top-down image
      uint32_t flushlen = 55 + imgsiz;
      // copy our frame, push it onto a stack to be flushed
      // asynchronously to sdcard
      uint8_t *flushbuf = new uint8_t[flushlen];
      memcpy(flushbuf, &flushlen, 4);  // write header length
      memcpy(flushbuf+4, &t.tv_sec, 4);
      memcpy(flushbuf+8, &t.tv_usec, 4);
      memcpy(flushbuf+12, &throttle_, 1);
      memcpy(flushbuf+13, &steering_, 1);
      memcpy(flushbuf+14, &accel_[0], 4);
      memcpy(flushbuf+14+4, &accel_[1], 4);
      memcpy(flushbuf+14+8, &accel_[2], 4);
      memcpy(flushbuf+26, &gyro_[0], 4);
      memcpy(flushbuf+26+4, &gyro_[1], 4);
      memcpy(flushbuf+26+8, &gyro_[2], 4);
      memcpy(flushbuf+38, &servo_pos_, 1);
      memcpy(flushbuf+39, wheel_pos_, 2*4);
      memcpy(flushbuf+47, wheel_dt_, 2*4);
      memcpy(flushbuf+55, topview, imgsiz);

      struct timeval t1;
      gettimeofday(&t1, NULL);
      float dt = t1.tv_sec - t.tv_sec + (t1.tv_usec - t.tv_usec) * 1e-6;
      if (dt > 0.1) {
        fprintf(stderr, "CameraThread::OnFrame: WARNING: "
            "alloc/copy took %fs\n", dt);
      }

      flush_thread_.AddEntry(output_fd_, flushbuf, flushlen);
      struct timeval t2;
      gettimeofday(&t2, NULL);
      dt = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) * 1e-6;
      if (dt > 0.1) {
        fprintf(stderr, "CameraThread::OnFrame: WARNING: "
            "flush_thread.AddEntry took %fs\n", dt);
      }
    }

    {
      static struct timeval t0 = {0, 0};
      float dt = t.tv_sec - t0.tv_sec + (t.tv_usec - t0.tv_usec) * 1e-6;
      if (dt > 0.1 && t0.tv_sec != 0) {
        fprintf(stderr, "CameraThread::OnFrame: WARNING: "
            "%fs gap between frames?!\n", dt);
      }
      t0 = t;
    }

    float u_a = throttle_ / 127.0;
    float u_s = steering_ / 127.0;
    float dt = t.tv_sec - last_t_.tv_sec + (t.tv_usec - last_t_.tv_usec) * 1e-6;
    controller_.UpdateState(config_, reprojected,
            u_a, u_s,
            accel_, gyro_,
            servo_pos_, wheel_pos_,
            dt, topview);  // annotate topview and display
    last_t_ = t;
    display_.UpdateBirdseye(topview, imgproc::uxsiz, imgproc::uysiz);

    if (autosteer_ && controller_.GetControl(config_, &u_a, &u_s, dt)) {
      steering_ = 127 * u_s;
      throttle_ = 127 * u_a;
      teensy.SetControls(frame_ & 4 ? 1 : 0, throttle_, steering_);
      // pca.SetPWM(PWMCHAN_STEERING, steering_);
      // pca.SetPWM(PWMCHAN_ESC, throttle_);
    }
  }

  bool autosteer_;
  DriveController controller_;
  DriverConfig config_;
  int frame_;

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

    int16_t *value = ((int16_t*) config_) + config_item_;

    switch(button) {
      case 'S': // start button: start recording
        if (!driver_.IsRecording()) {
          char fnamebuf[256];
          time_t start_time = time(NULL);
          struct tm start_time_tm;
          localtime_r(&start_time, &start_time_tm);
          strftime(fnamebuf, sizeof(fnamebuf), "cycloid-%Y%m%d-%H%M%S.rec",
              &start_time_tm);
          if (driver_.StartRecording(fnamebuf, 0)) {
            fprintf(stderr, "%d.%06d started recording %s\n",
                tv.tv_sec, tv.tv_usec, fnamebuf);
          }
        }
        break;
      case 'R':  // right trigger: stop recording
        if (driver_.IsRecording()) {
          driver_.StopRecording();
          fprintf(stderr, "%d.%06d stopped recording\n", tv.tv_sec, tv.tv_usec);
        }
        break;
      case 'L':
        if (!driver_.autosteer_) {
          fprintf(stderr, "%d.%06d autodrive ON\n", tv.tv_sec, tv.tv_usec);
          driver_.autosteer_ = true;
        }
        break;
      case 'B':
        driver_.controller_.ResetState();
        fprintf(stderr, "reset kalman filter\n");
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
        if (driver_.autosteer_) {
          driver_.autosteer_ = false;
          fprintf(stderr, "%d.%06d autodrive OFF\n", tv.tv_sec, tv.tv_usec);
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

  virtual void OnAxisMove(int axis, int16_t value) {
    switch (axis) {
      case 1:  // left stick y axis
        js_throttle_ = -value;
        break;
      case 2:  // right stick x axis
        js_steering_ = value;
        break;
    }
  }

  void UpdateDisplay() {
    // hack because all config values are int16_t's in 1/100th steps
    int16_t value = ((int16_t*) config_)[config_item_];
    // FIXME: does this work for negative values?
    fprintf(stderr, "%s %d.%02d\r", configmenu[config_item_],
        value / 100, value % 100);
  }
};

const char *DriverInputReceiver::configmenu[] = {
  "yellow thresh",
  "max speed",
  "max throttle",
  "traction limit",
  "steering kP",
  "steering kD",
  "brake kP",
  "motor kP",
  "motor kI",
  "motor kD",
  "motor offset",
  "lane offset",
  "lane turn-in"
};
const int DriverInputReceiver::N_CONFIGITEMS = sizeof(configmenu) / sizeof(configmenu[0]);

int main(int argc, char *argv[]) {
  signal(SIGINT, handle_sigint);

  feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

  int fps = 30;

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

  bool has_joystick = false;
  if (js.Open()) {
    has_joystick = true;
  } else {
    fprintf(stderr, "joystick not detected, but continuing anyway!\n");
  }

  teensy.Init();
  teensy.SetControls(0, 0, 0);
  teensy.GetFeedback(&servo_pos_, wheel_pos_, wheel_dt_);
  fprintf(stderr, "initial teensy state feedback: \n"
          "  servo %d encoders %d %d %d %d\r",
          servo_pos_, wheel_pos_[0], wheel_pos_[1],
          wheel_pos_[2], wheel_pos_[3]);

  // pca.Init(100);  // 100Hz output
  // pca.SetPWM(PWMCHAN_STEERING, 614);
  // pca.SetPWM(PWMCHAN_ESC, 614);

  imu.Init();

  struct timeval tv;
  gettimeofday(&tv, NULL);
  fprintf(stderr, "%d.%06d camera on @%d fps\n", tv.tv_sec, tv.tv_usec, fps);

  DriverInputReceiver input_receiver(&driver_.config_);
  if (!Camera::StartRecord(&driver_)) {
    return 1;
  }

  gettimeofday(&tv, NULL);
  fprintf(stderr, "%d.%06d started camera\n", tv.tv_sec, tv.tv_usec);

  while (!done) {
    int t = 0, s = 0;
    uint16_t b = 0;
    if (has_joystick && js.ReadInput(&input_receiver)) {
      if (!driver_.autosteer_) {
        // really need a better way to get this
        float steeroffset = driver_.controller_.ekf.GetState()[10];
        steering_ = -127 * clip(js_steering_ / 32767.0 - steeroffset, -1, 1);
        throttle_ = 127 * clip(js_throttle_ / 32767.0, -1, 1);
        // pca.SetPWM(PWMCHAN_STEERING, steering_);
        // pca.SetPWM(PWMCHAN_ESC, throttle_);
        teensy.SetControls(driver_.frame_ & 16 ? 1 : 0, throttle_, steering_);
      }
    }
    // FIXME: predict step here?
    {
      float temp;
      imu.ReadIMU(&accel_, &gyro_, &temp);
      // FIXME: imu EKF update step?
      teensy.GetFeedback(&servo_pos_, wheel_pos_, wheel_dt_);
    }
    usleep(1000);
  }

  Camera::StopRecord();
}
