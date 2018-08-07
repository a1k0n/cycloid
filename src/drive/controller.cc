#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

#include "drive/controller.h"

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::VectorXf;


DriveController::DriveController() {
  ResetState();
}

void DriveController::ResetState() {
  firstframe_ = true;
}

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

void DriveController::UpdateState(const DriverConfig &config,
    float throttle_in, float steering_in,
    const Vector3f &accel, const Vector3f &gyro,
    uint8_t servo_pos, const uint16_t *wheel_encoders, float dt) {

  if (firstframe_) {
    memcpy(last_encoders_, wheel_encoders, 4*sizeof(uint16_t));
    firstframe_ = false;
  }

  // average ds among wheel encoders which are actually moving
  float ds = 0, nds = 0;
  for (int i = 0; i < 4; i++) {  // only use rear encoders
    if (wheel_encoders[i] != last_encoders_[i]) {
      ds += (uint16_t) (wheel_encoders[i] - last_encoders_[i]);
      nds += 1;
    }
  }
  memcpy(last_encoders_, wheel_encoders, 4*sizeof(uint16_t));

  // update velocity estimate through crude filter
  velocity_ *= 0.9;
  if (nds > 0) {
    velocity_ += 0.1 * ds/(nds * dt);
  }
}

static float MotorControl_model(const DriverConfig &config,
    float v_target,
    float k1, float k2, float k3, float v) {
  float kP = (v_target >= v ? config.motor_kP : config.brake_kP) * 0.01;
  float accel = clip(kP * (v_target - v), -7, 7);
  // voltage (1 or 0)
  // v > 0 ????
  float V = accel > -k3 * v ? 1 : 0;
  // duty cycle
  float DC = clip(
      (accel + k3 * v) / (V*k1 - k2*v),
      0, config.max_throttle * 0.01);
  const float deadzone_min = config.motor_offset * 0.01;
  if (accel > 0 && DC < deadzone_min) {
    DC = deadzone_min;
  }
  return V == 1 ? DC : -DC;
}

static float MotorControl_model(float accel,
    float k1, float k2, float k3, float k4,
    float v) {
  if (v < 0.2) {
    k4 += k3;  // add static friction onto dynamic friction
  }
  float a_thresh = -k4;
  // voltage (1 or 0)
  float V = accel > a_thresh ? 1 : 0;
  // duty cycle
  float DC = clip(
      (accel + k4) / (V*k1 - k2*v),
      0, 1);
  return V == 1 ? DC : -DC;
}

bool DriveController::GetControl(const DriverConfig &config,
    float throttle_in, float steering_in,
    float *throttle_out, float *steering_out, float dt) {

  *throttle_out = throttle_in;
  *steering_out = steering_in;

  return true;
}

