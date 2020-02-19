#include "gpsdrive/gpsdrive.h"

#include <Eigen/Dense>

#include "hw/car/car.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "inih/ini.h"
#include "ui/display.h"

GPSDrive::GPSDrive(FlushThread *ft, IMU *imu, JoystickInput *js,
                   UIDisplay *disp)
    : flush_thread_(ft), imu_(imu), js_(js), display_(disp) {
  done_ = false;
}

bool GPSDrive::Init(const INIReader &ini) {
  if (config_.Load()) {
    fprintf(stderr, "Loaded driver configuration\n");
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

  return !done_;
}

void GPSDrive::OnDPadPress(char direction) {}
void GPSDrive::OnButtonPress(char button) {}
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
