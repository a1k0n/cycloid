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
      const Eigen::Vector3f &accel,
      const Eigen::Vector3f &gyro,
      uint8_t servo_pos,
      const uint16_t *wheel_encoders, float dt);

  void UpdateLocation(const DriverConfig &config,
          float x, float y, float theta);
  void AddSample(const DriverConfig &config,
          float x, float y, float theta);

  bool GetControl(const DriverConfig &config,
      float throttle_in, float steering_in,
      float *throttle_out, float *steering_out, float dt,
      bool autodrive, int frameno);

  void ResetState();

  int SerializedSize() const;
  int Serialize(uint8_t *buf, int buflen) const;
  void Dump() const;

  TrajectoryTracker *GetTracker() { return &track_; }

  // car state
  float x_, y_, theta_;
  float vf_, vr_;  // front and rear wheel velocity
  float w_;  // yaw rate
  float ierr_v_;  // integration error for velocity
  float ierr_w_;  // integration error for yaw rate
  float delta_;  // current steering angle

  float target_k_;  // mean path computation
  int k_samples_;

  float target_v_, target_w_;  // control targets
  float ye_, psie_, k_, vk_;  // relative trajectory target
  float bw_w_, bw_v_;  // control bandwidth for yaw and speed

  float cx_, cy_, nx_, ny_;

 private:
  TrajectoryTracker track_;
};

#endif  // DRIVE_CONTROLLER_H_
