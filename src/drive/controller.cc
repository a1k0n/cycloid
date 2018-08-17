#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "drive/controller.h"

using Eigen::Vector3f;

const float V_ALPHA = 0.1;

// circumference of tire (meters) / number of encoder ticks
const float V_SCALE = 0.04;  // 40cm circumference, 10 ticks

// servo closed loop response bandwidth (measured)
const float BW_SRV = 2*M_PI*4;

// FIXME(asloane): aren't these based on encoder ticks, not velocity?
const float M_K1 = 120.;  // DC motor response constants (measured)
const float M_K2 = 5.6;
const float M_K3 = 0.5;

DriveController::DriveController() {
  ResetState();
  if (!track_.LoadTrack("track.txt")) {
    fprintf(stderr, "***WARNING: NO TRACK LOADED; check track.txt***\n");
  }
}

void DriveController::ResetState() {
  velocity_ = 0;
  w_ = 0;
  ierr_v_ = 0;
  ierr_w_ = 0;
}

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

void DriveController::UpdateState(const DriverConfig &config,
    float throttle_in, float steering_in,
    const Vector3f &accel, const Vector3f &gyro,
    uint8_t servo_pos, const uint16_t *wheel_delta, float dt) {

  // average ds among wheel encoders which are actually moving
  float ds = 0, nds = 0;
  for (int i = 0; i < 4; i++) {
    if (wheel_delta[i] != 0) {
      ds += wheel_delta[i];
      nds += 1;
    }
  }

  // update velocity estimate through crude filter
  velocity_ *= (1 - V_ALPHA);
  if (nds > 0) {
    velocity_ += V_ALPHA * V_SCALE * ds/(nds * dt);
  }

  w_ = gyro[2];
}

bool DriveController::GetControl(const DriverConfig &config,
      float throttle_in, float steering_in,
    float *throttle_out, float *steering_out, float dt) {

  // okay, let's control for yaw rate!
  // throttle_in controls vmax (w.r.t. the configured value)
  // steering_in controls desired curvature

  // if we're braking or coasting, just control that manually
  if (throttle_in <= 0) {
    *throttle_out = throttle_in;
    *steering_out = -steering_in;  // yaw is backwards
    return true;
  }

  // max curvature is 1m radius
  float k = -steering_in * 1;

  float vmax = throttle_in * config.speed_limit * 0.01;
  float kmin = config.traction_limit * 0.01 / (vmax*vmax);

  float target_v = vmax;
  if (fabs(k) > kmin) {  // any curvature more than this will reduce speed
    target_v = sqrt(config.traction_limit * 0.01 / fabs(k));
  }

  // use average of target velocity and current velocity to determine
  // target yaw rate
  float target_w = k*(velocity_ + target_v)*0.5;

  float err_v = velocity_ - target_v;
  float err_w = w_ - target_w;

  float BW_w = 2*M_PI*0.01*config.yaw_bw;
  *steering_out = clip(-BW_w/target_v * (ierr_w_ + err_w / BW_SRV), -1, 1);
  // printf("w control: w=%f/%f err_w=%f ierr_w=%f out=%f\n",
  //     w_, target_w, err_w, ierr_w_, *steering_out);

  float BW_v = 2*M_PI*0.01*config.motor_bw;
  float Kp = BW_v / (M_K1 - M_K2*velocity_);
  float Ki = M_K3;
  *throttle_out = clip(-Kp*(err_v + Ki*ierr_v_), 0, 1);
  if (*throttle_out == 0 && velocity_ > 0) {  // handle braking
    // alternate control law
    float Kp2 = BW_v / (-M_K2*velocity_);
    *throttle_out = clip(Kp2*(err_v + Ki*ierr_v_), -1, 0);
    // printf("v brake: v=%f/%f Kp=%f Ki=%f err_v=%f ierr_v=%f out=%f\n",
    //     velocity_, target_v, Kp, Ki, err_v, ierr_v_, *throttle_out);
  } else {
    // printf("v control: v=%f/%f Kp=%f Ki=%f err_v=%f ierr_v=%f out=%f\n",
    //     velocity_, target_v, Kp, Ki, err_v, ierr_v_, *throttle_out);
  }

  ierr_v_ += dt*err_v;
  if ((*steering_out > -1 && *steering_out < 1) ||
      (err_w > 0 && ierr_w_ < 0) || (err_w < 0 && ierr_w_ > 0)) {
    // don't windup integrator if we're maxed out
    ierr_w_ += dt*err_w;
    ierr_w_ += dt*err_w;
  }

  return true;
}

