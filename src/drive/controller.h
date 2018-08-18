#ifndef DRIVE_CONTROLLER_H_
#define DRIVE_CONTROLLER_H_

#include <Eigen/Dense>
#include <math.h>

#include "drive/config.h"
#include "drive/trajtrack.h"

class DriveController {
 public:
  DriveController();

  void UpdateState(const DriverConfig &config,
      float throttle_in, float steering_in,
      const Eigen::Vector3f &accel,
      const Eigen::Vector3f &gyro,
      uint8_t servo_pos,
      const uint16_t *wheel_encoders, float dt);

  void UpdateLocation(float x, float y, float theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
  }

  bool GetControl(const DriverConfig &config,
      float throttle_in, float steering_in,
      float *throttle_out, float *steering_out, float dt,
      bool autodrive);

  void ResetState();

  TrajectoryTracker *GetTracker() { return &track_; }

 private:
  float TargetCurvature(const DriverConfig &config);

  // car state
  float x_, y_, theta_;
  float velocity_;  // forward velocity
  float w_;  // yaw rate
  float ierr_v_;  // integration error for velocity
  float ierr_w_;  // integration error for yaw rate
  TrajectoryTracker track_;
};

#endif  // DRIVE_CONTROLLER_H_
