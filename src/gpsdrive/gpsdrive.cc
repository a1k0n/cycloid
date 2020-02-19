#include "gpsdrive/gpsdrive.h"

#include <sys/time.h>

#include <Eigen/Dense>

#include "hw/car/car.h"
#include "hw/gps/ubx.h"
#include "hw/imu/mag.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "inih/ini.h"
#include "ui/display.h"

GPSDrive::GPSDrive(FlushThread *ft, IMU *imu, Magnetometer *mag,
                   JoystickInput *js, UIDisplay *disp)
    : flush_thread_(ft), imu_(imu), mag_(mag), js_(js), display_(disp) {
  done_ = false;
  record_fp_ = NULL;
  js_throttle_ = 0;
  js_steering_ = 0;
  pthread_mutex_init(&record_mut_, NULL);
}

void* GPSDrive::gpsThread(void* arg) {
  GPSDrive *drive = (GPSDrive*)arg;
  fprintf(stderr, "GPS receive thread started\n");
  ubx_read_loop(drive->ubx_fd_, drive);
  return NULL;
}

bool GPSDrive::Init(const INIReader &ini) {
  if (config_.Load()) {
    fprintf(stderr, "Loaded driver configuration\n");
  }

  ubx_fd_ = ubx_open();
  if (ubx_fd_ == -1) {
    return false;
  }

  if (pthread_create(&gps_thread_, NULL, GPSDrive::gpsThread, (void*) this)) {
    perror("pthread_create");
    return false;
  }

  // draw UI screen
  display_->UpdateStatus("GPSDrive started.");

  return true;
}

GPSDrive::~GPSDrive() {}

bool GPSDrive::OnControlFrame(CarHW *car, float dt) {
  if (js_) {
    js_->ReadInput(this);
  }

  Eigen::Vector3f accel, gyro, mag;
  if (!imu_->ReadIMU(&accel, &gyro)) {
    fprintf(stderr, "imu read failure\n");
    accel = accel.Zero();
    gyro = gyro.Zero();
  }
  if (!mag_->ReadMag(&mag)) {
    fprintf(stderr, "magnetometer read failure\n");
    mag = mag.Zero();
  }

  car->SetControls(2, js_throttle_ / 32768.0, js_steering_ / 32768.0);

  float ds, v;
  car->GetWheelMotion(&ds, &v);

  if (record_fp_ != NULL) {
    timeval tv;
    gettimeofday(&tv, NULL);
    pthread_mutex_lock(&record_mut_);
    fprintf(record_fp_,
            "%ld.%06ld control %d %d wheel %f %f imu %f %f %f %f %f %f mag %f "
            "%f %f\n",
            tv.tv_sec, tv.tv_usec, js_throttle_, js_steering_, ds, v, accel[0],
            accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1],
            mag[2]);
    pthread_mutex_unlock(&record_mut_);
  }

  return !done_;
}

void GPSDrive::OnNav(const nav_pvt &msg) {
  if (record_fp_ != NULL) {
    timeval tv;
    gettimeofday(&tv, NULL);
    pthread_mutex_lock(&record_mut_);
    fprintf(record_fp_, "%ld.%06ld gps ", tv.tv_sec, tv.tv_usec);
    fprintf(record_fp_, "%04d-%02d-%02dT%02d:%02d:%02d.%09d ", msg.year, msg.month, msg.day,
           msg.hour, msg.min, msg.sec, msg.nano);
    fprintf(record_fp_,
        "fix:%d numSV:%d %d.%07d +-%dmm %d.%07d +-%dmm height %dmm "
        "vel %d %d %d +-%d mm/s "
        "heading motion %d.%05d vehicle %d +- %d.%05d\n",
        msg.fixType, msg.numSV, msg.lon / 10000000,
        std::abs(msg.lon) % 10000000, msg.hAcc, msg.lat / 10000000,
        std::abs(msg.lat) % 10000000, msg.vAcc, msg.height, msg.velN, msg.velE,
        msg.velD, msg.sAcc, msg.headMot / 100000,
        std::abs(msg.headMot) % 100000, msg.headVeh, msg.headAcc / 100000,
        msg.headAcc % 100000);
    pthread_mutex_unlock(&record_mut_);
  }
}

void GPSDrive::OnDPadPress(char direction) {}
void GPSDrive::OnButtonPress(char button) {
  switch (button) {
    case '+':  // start button
      StartRecording();
    case '-':  // stop button
      StopRecording();
  }
}

void GPSDrive::OnButtonRelease(char button) {}

void GPSDrive::OnAxisMove(int axis, int16_t value) {
  switch (axis) {
    case 1:  // left stick y axis
      js_throttle_ = -value;
      break;
    case 2:  // right stick x axis
      js_steering_ = value;
      break;
  }
}

void GPSDrive::StartRecording() {
  timeval tv;
  gettimeofday(&tv, NULL);

  if (record_fp_ != NULL) {
    return;
  }

  char fnamebuf[256];
  time_t start_time = time(NULL);
  struct tm start_time_tm;
  localtime_r(&start_time, &start_time_tm);
  strftime(fnamebuf, sizeof(fnamebuf), "gpsdrive-%Y%m%d-%H%M%S.log",
           &start_time_tm);
  record_fp_ = fopen(fnamebuf, "w");
  if (!record_fp_) {
    perror(fnamebuf);
  }

  printf("%ld.%06ld start recording %s\n", tv.tv_sec, tv.tv_usec, fnamebuf);
  display_->UpdateStatus(fnamebuf);
}

void GPSDrive::StopRecording() {
  timeval tv;
  gettimeofday(&tv, NULL);

  if (record_fp_ == NULL) {
    return;
  }

  pthread_mutex_lock(&record_mut_);
  record_fp_ = NULL;
  fclose(record_fp_);
  pthread_mutex_unlock(&record_mut_);

  printf("%ld.%06ld stop recording\n", tv.tv_sec, tv.tv_usec);
  display_->UpdateStatus("stop recording");
}

void GPSDrive::Quit() {
  done_ = true;
  StopRecording();
}