#include "gpsdrive/gpsdrive.h"

#include <sys/time.h>

#include <Eigen/Dense>

#include "hw/car/car.h"
#include "hw/gps/ubx.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "inih/ini.h"
#include "ui/display.h"

GPSDrive::GPSDrive(FlushThread *ft, IMU *imu, JoystickInput *js,
                   UIDisplay *disp)
    : flush_thread_(ft), imu_(imu), js_(js), display_(disp) {
  done_ = false;
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

  js_throttle_ = 0;
  js_steering_ = 0;

  return true;
}

GPSDrive::~GPSDrive() {}

bool GPSDrive::OnControlFrame(CarHW *car, float dt) {
  if (js_) {
    js_->ReadInput(this);
  }

  Eigen::Vector3f accel, gyro;
  if (!imu_->ReadIMU(&accel, &gyro)) {
    fprintf(stderr, "imu read failure\n");
    accel = accel.Zero();
    gyro = gyro.Zero();
  }

  car->SetControls(2, js_throttle_ / 32768.0, js_steering_ / 32768.0);

  float ds, v;
  car->GetWheelMotion(&ds, &v);

  timeval tv;
  gettimeofday(&tv, NULL);
  printf("%ld.%06ld control %d %d wheel %f %f imu %f %f %f %f %f %f\n", tv.tv_sec,
         tv.tv_usec, js_throttle_, js_steering_, ds, v, accel[0], accel[1], accel[2],
         gyro[0], gyro[1], gyro[2]);

  return !done_;
}

void GPSDrive::OnNav(const nav_pvt &msg) {
  timeval tv;
  gettimeofday(&tv, NULL);
  printf("%ld.%06ld gps ", tv.tv_sec, tv.tv_usec);
  printf("%04d-%02d-%02dT%02d:%02d:%02d.%09d ", msg.year, msg.month, msg.day,
         msg.hour, msg.min, msg.sec, msg.nano);
  printf(
      "fix:%d numSV:%d %d.%07d +-%dmm %d.%07d +-%dmm height %dmm "
      "vel %d %d %d +-%d mm/s "
      "heading motion %d.%05d vehicle %d +- %d.%05d\n",
      msg.fixType, msg.numSV, msg.lon / 10000000, std::abs(msg.lon) % 10000000,
      msg.hAcc, msg.lat / 10000000, std::abs(msg.lat) % 10000000, msg.vAcc,
      msg.height, msg.velN, msg.velE, msg.velD, msg.sAcc, msg.headMot / 100000,
      std::abs(msg.headMot) % 100000, msg.headVeh, msg.headAcc / 100000,
      msg.headAcc % 100000);
}

void GPSDrive::OnDPadPress(char direction) {}
void GPSDrive::OnButtonPress(char button) {
  timeval tv;
  gettimeofday(&tv, NULL);

  switch (button) {
    case '+':  // start button
      printf("%ld.%06ld start\n", tv.tv_sec, tv.tv_usec);
      break;
    case '-':  // stop button
      printf("%ld.%06ld stop\n", tv.tv_sec, tv.tv_usec);
      break;
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
