#ifndef DRIVE_CONTROLLER_H_
#define DRIVE_CONTROLLER_H_

#include <Eigen/Dense>
#include <math.h>

#include "drive/config.h"

class DriveController {
 public:
  DriveController();

  // do full kalman filter update: prediction and sensor fusion
  void UpdateState(const DriverConfig &config,
      float throttle_in, float steering_in,
      const Eigen::Vector3f &accel,
      const Eigen::Vector3f &gyro,
      uint8_t servo_pos,
      const uint16_t *wheel_encoders, float dt);

  bool GetControl(const DriverConfig &config,
      float throttle_in, float steering_in,
      float *throttle_out, float *steering_out, float dt);

  void ResetState();

 private:
  // controller state
  float velocity_;  // forward velocity
  float w_;  // yaw rate
  float ierr_v_;  // integration error for velocity
  float ierr_w_;  // integration error for yaw rate
};

#endif  // DRIVE_CONTROLLER_H_
