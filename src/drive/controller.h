#ifndef DRIVE_CONTROLLER_H_
#define DRIVE_CONTROLLER_H_

#include <math.h>
#include <Eigen/Dense>

#include "drive/config.h"
#include "drive/vflookup.h"

static const int kLookaheadAngles = 20;

class DriveController {
 public:
  DriveController();

  void UpdateState(const DriverConfig &config, const Eigen::Vector3f &accel,
                   const Eigen::Vector3f &gyro, float wheel_v, float dt);

  void UpdateLocation(const DriverConfig &config, const float *xytheta);

  void Plan(const DriverConfig &config);

  bool GetControl(const DriverConfig &config,
      float throttle_in, float steering_in,
      float *throttle_out, float *steering_out, float dt,
      bool autodrive, int frameno);

  void ResetState();

  int SerializedSize() const;
  int Serialize(uint8_t *buf, int buflen) const;
  void Dump() const;

  // car state
  float x_, y_, theta_;
  float vf_, vr_;        // front and rear wheel velocity
  float w_;              // yaw rate
  float prev_throttle_;  // previous throttle control
  float prev_steer_;     // previous steer control
  float ierr_v_;         // integrated velocity error
  float ierr_k_;         // integrated curvature error

  float target_ks_[1+kLookaheadAngles*2];    // next potential control actions
  float target_k_Vs_[1+kLookaheadAngles*2];  // total value of each action over all particles
  float target_k_;

  float target_v_, target_w_;  // control targets
  float bw_w_, bw_v_;          // control bandwidth for yaw and speed

 private:
  ValueFuncLookup V_;
};

#endif  // DRIVE_CONTROLLER_H_
