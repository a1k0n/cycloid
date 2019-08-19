#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>

#include "drive/controller.h"

using Eigen::Vector3f;

const int STEER_LIMIT_LOW = -85;
const int STEER_LIMIT_HIGH = 127;

DriveController::DriveController() {
  ResetState();
  if (!V_.Init()) {
    perror("*** WARNING: no vf.bin (value function) found, cannot autodrive!");
  }
}

void DriveController::ResetState() {
  vr_ = vf_ = 0;
  w_ = 0;
  prev_throttle_ = 0;
  prev_steer_ = 0;
  ierr_v_ = 0;
  ierr_k_ = 0;
}

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

void DriveController::UpdateState(const DriverConfig &config,
                                  const Vector3f &accel, const Vector3f &gyro,
                                  float wheel_v, float dt) {
  vr_ = vf_ = wheel_v;
  w_ = gyro[2];
}

void DriveController::UpdateLocation(const DriverConfig &config, const float *xytheta) {
  x_ = xytheta[0];
  y_ = xytheta[1];
  theta_ = xytheta[2];
}

void DriveController::Plan(const DriverConfig &config) {
  // TODO(asloane): consider more feasible maneuvers
  // curvature can swing about 0.3 1/m from wherever it is now in 10ms
  const int nangles = kLookaheadAngles;
  float k0 = 0;
  float dk = config.lookahead_krate * 0.01 / nangles;

/*
  if (vr_ > 0.3) {
    k0 = clip(w_ / vr_, -kmax + 3 * dk, kmax - 3 * dk);
  }
  */

  float s = config.lookahead_dist * 0.01;
  for (int a = 0; a < (1+nangles*2); a++) {
    // compute next x, y, theta
    float k = k0 - (a-nangles)*dk;
    float ks = k * s;
    float V = 0;
    float P = 0;
    {
      // jump ahead 30ms before we start planning
      float lookaheadx = 0.090 * vr_;
      float x0 = x_, y0 = y_, t0 = theta_;
      x0 += lookaheadx*cos(t0);
      y0 += lookaheadx*sin(t0);
      float theta1 = t0 + ks;
      float C = cos(t0), S = sin(t0);
      if (fabs(k) < 1e-2)
        k = 1e-2;  // hack: avoid special cases for zero curvature

      // add up the path cost every 10cm
      for (float st = 0.30; st < s; st += 0.30) {
        float xt = x0 + (sin(t0 + k*st) - S) / k;
        float yt = y0 + (C - cos(t0 + k*st)) / k;
        P += V_.C(xt, yt) * config.path_penalty * 0.001;
      }
      // and then the final cost-to-go
      float x1 = x0 + (sin(t0 + ks) - S) / k;
      float y1 = y0 + (C - cos(t0 + ks)) / k;
      V += V_.V(x1, y1, theta1);
    }
    target_ks_[a] = k;
    target_k_Vs_[a] = V+P;
    // printf("%0.1f/%0.1f ", V, P);
  }

  // compute target curvature at all times, just for datalogging purposes
  float bestV = target_k_Vs_[0];
  target_k_ = target_ks_[0];
  for (int a = 1; a < (nangles * 2 + 1); a++) {
    float V = target_k_Vs_[a];
    if (V < bestV) {
      bestV = V;
      target_k_ = target_ks_[a];
    }
  }
}

bool DriveController::GetControl(const DriverConfig &config,
    float throttle_in, float steering_in,
    float *throttle_out, float *steering_out, float dt,
    bool autodrive, int frameno) {

  // okay, let's control for yaw rate!
  // throttle_in controls vmax (w.r.t. the configured value)
  // steering_in controls desired curvature

  float autok = target_k_;

  // if we're braking or coasting, just control that manually
  if (!autodrive && throttle_in <= 0) {
    *throttle_out = throttle_in;
    // yaw is backwards
    *steering_out =
        clip(steering_in, STEER_LIMIT_LOW / 127.0, STEER_LIMIT_HIGH / 127.0);
    prev_steer_ = *steering_out;
    prev_throttle_ = *throttle_out;
    ierr_v_ = 0;
    ierr_k_ = 0;
    return true;
  }

  // max curvature is servo_rate
  // use a quadratic curve to give finer control near center
  float k = steering_in * config.servo_rate * 0.01 * fabs(steering_in);
  float vk = k;  // curvature to use for velocity calc
  float vmax = throttle_in * config.speed_limit * 0.01;
  if (autodrive) {
    k = autok;
    // use lookahead vk_ to slow down early
    // vk = fmax(fabs(vk_), fabs(k));
    // maybe just use vk_ directly here? then we speed up at corner exit
    vk = fabs(k);
    vmax = config.speed_limit * 0.01;
  }

  float kmin = config.traction_limit * 0.01 / (vmax*vmax);

  float target_v = vmax;
  if (fabs(vk) > kmin) {  // any curvature more than this will reduce speed
    target_v = sqrt(config.traction_limit * 0.01 / fabs(vk));
  }

  // sometimes the controller gives infeasible curvatures; clamp them
  float target_k = clip(k, -2, 2);
  float target_w = k*vr_;

  float BW_w = 100. / config.servo_rate;
  float srv_off = 0.01 * config.servo_offset;
  float srv_fine = 0.01 * config.servo_finetune;
  if (vr_ > 0.2) {
    float kerr = target_k - w_/vr_;
    ierr_k_ = clip(ierr_k_ + dt * srv_fine * kerr, -2, 2);
  } else {
    ierr_k_ = 0;
  }
  *steering_out = clip((target_k - srv_off) * BW_w + ierr_k_,
                       STEER_LIMIT_LOW / 127.0, STEER_LIMIT_HIGH / 127.0);
  prev_steer_ = *steering_out;

  float vgain = 0.01 * config.motor_gain;
  float kI = 0.01 * config.motor_kI;
  // boost control gain at high velocities
  // ...or don't, we need to prevent oscillation
  // vgain = clip(vgain / (1 - 0.025*vr_), 0.01, 2.0);
  float verr = target_v - vr_;
  float u = vgain * (verr + kI*ierr_v_);
  if (u > -1 && u < 1) {
    ierr_v_ += verr * dt;
  }
  *throttle_out = clip(u, -1, 1);
  prev_throttle_ = *throttle_out;

  // heuristic: subtract magnitude of yaw rate error from throttle control
  float werr = fabsf(target_w - w_) * 0.01 * config.turnin_lift;
  *throttle_out = clip(*throttle_out - werr, -1, 1);

  // update state for datalogging
  target_v_ = target_v;
  target_w_ = target_w;
  bw_w_ = BW_w;
  bw_v_ = vgain;

  return true;
}

int DriveController::SerializedSize() const {
  return 8 + sizeof(float) * (12 + 2 * (1+kLookaheadAngles*2));
}

int DriveController::Serialize(uint8_t *buf, int buflen) const {
  uint32_t len = SerializedSize();
  assert(buflen >= static_cast<int>(len));

  memcpy(buf, "CTL2", 4);  // controller state
  memcpy(buf + 4, &len, 4);
  buf += 8;

  memcpy(buf, &x_, 4);
  buf += 4;
  memcpy(buf, &y_, 4);
  buf += 4;
  memcpy(buf, &theta_, 4);
  buf += 4;
  memcpy(buf, &vr_, 4);
  buf += 4;
  memcpy(buf, &w_, 4);
  buf += 4;
  memcpy(buf, &prev_steer_, 4);
  buf += 4;
  memcpy(buf, &prev_throttle_, 4);
  buf += 4;
  memcpy(buf, &ierr_k_, 4);
  buf += 4;
  memcpy(buf, &target_v_, 4);
  buf += 4;
  memcpy(buf, &target_w_, 4);
  buf += 4;
  memcpy(buf, &bw_w_, 4);
  buf += 4;
  memcpy(buf, &bw_v_, 4);
  buf += 4;
  for (int a = 0; a < (1+kLookaheadAngles*2); a++) {
    memcpy(buf, &target_ks_[a], 4);
    buf += 4;
  }
  for (int a = 0; a < (1+kLookaheadAngles*2); a++) {
    memcpy(buf, &target_k_Vs_[a], 4);
    buf += 4;
  }

  return len;
}

void DriveController::Dump() const {
  printf("deprecated");
}
