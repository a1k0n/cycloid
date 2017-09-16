#ifndef DRIVE_CONTROLLER_H_
#define DRIVE_CONTROLLER_H_

#include <Eigen/Dense>
#include <math.h>

#include "drive/config.h"
#include "drive/ekf.h"

class DriveController {
 public:
  DriveController();

  // do full kalman filter update: prediction and sensor fusion
  void UpdateState(const DriverConfig &config, int32_t *reprojected,
      float throttle_in, float steering_in,
      const Eigen::Vector3f &accel,
      const Eigen::Vector3f &gyro,
      uint8_t servo_pos,
      const uint16_t *wheel_encoders, float dt,
      uint8_t *annotatedyuv);

  bool GetControl(const DriverConfig &config,
      float *throttle_out, float *steering_out, float dt);

  void ResetState();

  // update w/ slope and intercept of line on ground
  void UpdateCamera(const DriverConfig &config, int32_t *reprojected,
      uint8_t *annotated);

  EKF ekf;

  bool firstframe_;
  uint16_t last_encoders_[4];
};

#endif  // DRIVE_CONTROLLER_H_
