#include <fcntl.h>
#include <fenv.h>
#include <getopt.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <deque>

#include "cam/cam.h"
// #include "car/pca9685.h"
#include "car/teensy.h"
#include "drive/controller.h"
#include "drive/imgproc.h"
#include "gpio/i2c.h"
#include "imu/imu.h"
#include "input/js.h"

volatile bool done = false;
int8_t throttle_ = 0, steering_ = 0;

const int PWMCHAN_STEERING = 14;
const int PWMCHAN_ESC = 15;

void handle_sigint(int signo) { done = true; }

I2C i2c;
// PCA9685 pca(i2c);
Teensy teensy(i2c);
IMU imu(i2c);
Eigen::Vector3f accel_(0, 0, 0), gyro_(0, 0, 0);
uint8_t servo_pos_ = 110;
uint16_t wheel_pos_[4] = {0, 0, 0, 0};
uint16_t wheel_dt_[4] = {0, 0, 0, 0};

// asynchronous flush to sdcard
struct FlushEntry {
  int fd_;
  uint8_t *buf_;
  size_t len_;
  size_t unsynced_;

  FlushEntry() { buf_ = NULL; }
  FlushEntry(int fd, uint8_t *buf, size_t len):
    fd_(fd), buf_(buf), len_(len) { unsynced_ = 0; }

  void flush() {
    if (len_ == -1) {
      fprintf(stderr, "FlushThread: closing fd %d\n", fd_);
      close(fd_);
    }
    if (buf_ != NULL) {
      if (write(fd_, buf_, len_) != len_) {
        perror("FlushThread write");
      }
      delete[] buf_;
      buf_ = NULL;
      unsynced_ += len_;
      // sync every 1MB
      // way too expensive! wtf!
      if (unsynced_ > 1048576) {
        unsynced_ = 0;
        fsync(fd_);
      }
    }
  }
};

class FlushThread {
 public:
  FlushThread() {
    pthread_mutex_init(&mutex_, NULL);
    sem_init(&sem_, 0, 0);
  }

  ~FlushThread() {
    // terminate the thread?
  }

  bool Init() {
    if (pthread_create(&thread_, NULL, thread_entry, this) != 0) {
      perror("FlushThread: pthread_create");
      return false;
    }
    return true;
  }

  void AddEntry(int fd, uint8_t *buf, size_t len) {
    static int count = 0;
    pthread_mutex_lock(&mutex_);
    flush_queue_.push_back(FlushEntry(fd, buf, len));
    size_t siz = flush_queue_.size();
    pthread_mutex_unlock(&mutex_);
    sem_post(&sem_);
    count++;
    if (count >= 15) {
      if (siz > 2) {
        fprintf(stderr, "[FlushThread %d]\r", siz);
        fflush(stderr);
      }
      count = 0;
    }
#if 0
    int semval;
    sem_getvalue(&sem_, &semval);
    fprintf(stderr, "Flusher: qsize %d sem %d\n", flush_queue_.size(), semval);
#endif
  }

 private:
  static void* thread_entry(void* arg) {
    FlushThread *self = reinterpret_cast<FlushThread*>(arg);

    fprintf(stderr, "FlushThread: started\n");

    for (;;) {
      sem_wait(&self->sem_);
      pthread_mutex_lock(&self->mutex_);
      if (!self->flush_queue_.empty()) {
        FlushEntry e = self->flush_queue_.front();
        self->flush_queue_.pop_front();
        pthread_mutex_unlock(&self->mutex_);
        e.flush();
      } else {
        pthread_mutex_unlock(&self->mutex_);
      }
    }
  }

  std::deque<FlushEntry> flush_queue_;
  pthread_mutex_t mutex_;
  pthread_t thread_;
  sem_t sem_;
};

FlushThread flush_thread_;

class Driver: public CameraReceiver {
 public:
  Driver() {
    output_fd_ = -1;
    frame_ = 0;
    frameskip_ = 0;
    autosteer_ = false;
    gettimeofday(&last_t_, NULL);
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
    if (IsRecording() && frame_ > frameskip_) {
      frame_ = 0;
      // save reprojected low-res top-down image
      uint32_t imgsiz = imgproc::uxsiz * imgproc::uysiz * 3;
      uint32_t flushlen = 55 + imgsiz;
      // convert 32-bit buf down to 8 bits for recording
      uint8_t topview[imgsiz];
      for (int i = 0; i < imgsiz; i++) {
        topview[i] = reprojected[i];
      }
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
    controller_.UpdateState(reprojected,
            u_a, u_s,
            accel_, gyro_,
            servo_pos_, wheel_pos_,
            dt);
    last_t_ = t;

    if (autosteer_ && controller_.GetControl(&u_a, &u_s, dt)) {
      steering_ = 127 * u_s;
      throttle_ = 127 * u_a;
      teensy.SetControls(frame_ & 4 ? 1 : 0, throttle_, steering_);
      // pca.SetPWM(PWMCHAN_STEERING, steering_);
      // pca.SetPWM(PWMCHAN_ESC, throttle_);
    }
  }

  bool autosteer_;
  DriveController controller_;
  int frame_;

 private:
  int output_fd_;
  int frameskip_;
  struct timeval last_t_;
};

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

int main(int argc, char *argv[]) {
  signal(SIGINT, handle_sigint);

  feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

  int fps = 30;
  int frameskip = 0;

  if (!flush_thread_.Init()) {
    return 1;
  }

  if (!Camera::Init(640, 480, fps))
    return 1;

  JoystickInput js;

  if (!i2c.Open()) {
    return 1;
  }

  bool has_joystick = false;
  if (js.Open()) {
    has_joystick = true;
  } else {
    fprintf(stderr, "joystick not detected!\n");
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

  Driver driver;
  if (!Camera::StartRecord(&driver)) {
    return 1;
  }

  gettimeofday(&tv, NULL);
  fprintf(stderr, "%d.%06d started camera\n", tv.tv_sec, tv.tv_usec);

  while (!done) {
    int t = 0, s = 0;
    uint16_t b = 0;
    if (has_joystick && js.ReadInput(&t, &s, &b)) {
      gettimeofday(&tv, NULL);
      if ((b & 0x40) && !driver.IsRecording()) {  // start button: start recording
        char fnamebuf[256];
        time_t start_time = time(NULL);
        struct tm start_time_tm;
        localtime_r(&start_time, &start_time_tm);
        strftime(fnamebuf, sizeof(fnamebuf), "cycloid-%Y%m%d-%H%M%S.rec", &start_time_tm);
        if (driver.StartRecording(fnamebuf, frameskip)) {
          fprintf(stderr, "%d.%06d started recording %s %d/%d fps\n", tv.tv_sec, tv.tv_usec, fnamebuf, fps, frameskip+1);
        }
      }
      if (b & 0x20 && driver.IsRecording()) {
        driver.StopRecording();
        fprintf(stderr, "%d.%06d stopped recording\n", tv.tv_sec, tv.tv_usec);
      }

      if (b & 0x10) {  // not sure which button this is
        if (!driver.autosteer_) {
          fprintf(stderr, "%d.%06d autosteer ON\n", tv.tv_sec, tv.tv_usec);
          driver.autosteer_ = true;
        }
      } else {
        if (driver.autosteer_) {
          driver.autosteer_ = false;
          fprintf(stderr, "%d.%06d autosteer OFF\n", tv.tv_sec, tv.tv_usec);
        }
      }

      if (b & 0x01) {
        driver.controller_.ResetState();
        fprintf(stderr, "reset kalman filter\n");
      }

      if (!driver.autosteer_) {
        // really need a better way to get this
        float steeroffset = driver.controller_.ekf.GetState()[10];
        steering_ = clip(-127*(s / 32767.0 - steeroffset), -127, 127);
        throttle_ = clip(127*t / 32767.0, -127, 127);
        // pca.SetPWM(PWMCHAN_STEERING, steering_);
        // pca.SetPWM(PWMCHAN_ESC, throttle_);
        teensy.SetControls(driver.frame_ & 16 ? 1 : 0, throttle_, steering_);
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
