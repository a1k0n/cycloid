#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "drive/controller.h"

using Eigen::Vector3f;

const float V_ALPHA = 0.3;

// servo closed loop response bandwidth (measured)
const float BW_SRV = 0.2*M_PI*4;  // Hz

const float SRV_A = -67.161224 / 127.0;
const float SRV_B = -3.835942 / 127.0;
const float SRV_C = 0.016797 / 127.0;
const float SRV_D = 10.778385 / 127.0;

const float M_K1 = 13.7;  // DC motor response constants (measured)
const float M_K2 = 1.26;
const float M_K3 = 0.53;
const float M_OFFSET = 0.103;  // minimum control input (dead zone)

const float GEOM_LF = 6.5*.0254;  // car geometry; A = CG to front axle length
const float GEOM_LR = 5*.0254;  // CG to rear axle (m)

const int STEER_LIMIT_LOW = -85;
const int STEER_LIMIT_HIGH = 127;

DriveController::DriveController() {
  ResetState();
  if (!track_.LoadTrack("track.txt")) {
    fprintf(stderr, "***WARNING: NO TRACK LOADED; check track.txt***\n");
  }
}

void DriveController::ResetState() {
  vr_ = vf_ = 0;
  w_ = 0;
  prev_throttle_ = 0;
  prev_steer_ = 0;
  prev_v_err_ = 0;
  prev_k_err_ = 0;
}

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

void DriveController::UpdateState(const DriverConfig &config,
    const Vector3f &accel, const Vector3f &gyro,
    uint8_t servo_pos, const uint16_t *wheel_period, float dt) {

  // with new STM32 hat firmware, this isn't necessary and we have a better
  // wheel speed estimate
  if (wheel_period[0] == 0) {
    vf_ = vr_ = 0;
  } else {
    vf_ = vr_ = V_SCALE * 1e6 / wheel_period[0];
  }

  w_ = gyro[2];
}

void DriveController::UpdateLocation(const DriverConfig &config,
    float x, float y, float theta) {
  x_ = x;
  y_ = y;
  // NOTE: theta is meaningless here; I shouldn't be tracking it
  theta_ = theta;
  if (!track_.GetTarget(x_, y_, config.lookahead,
        &cx_, &cy_, &nx_, &ny_, &k_, &vk_)) {
    cx_ = cy_ = ny_ = 0;
    nx_ = 1;
    // circle left if you're confused
    k_ = 2;
    vk_ = 1;
  }
  ye_ = 0;
  psie_ = 0;
  target_k_ = 0;
  k_samples_ = 0;
}

void DriveController::AddSample(const DriverConfig &config,
    float x, float y, float theta) {
  // (nx_, ny_) is the vector pointing towards +y (left)
  float ye = ((x - cx_)*nx_ + (y - cy_)*ny_);

  float C = cos(theta), S = sin(theta);

  // the car's "y" coordinate is (-S, C); measure cos/sin psi
  float Cp = -S*nx_ + C*ny_;
  float Sp = S*ny_ + C*nx_;
  float Cpy = Cp / (1 - k_ * ye);

  float Kpy = config.steering_kpy * 0.01;
  float Kvy = config.steering_kvy * 0.01;
  float targetk = Cpy*(ye*Cpy*(-Kpy*Cp) + Sp*(k_*Sp - Kvy*Cp) + k_);

  // update control state for datalogging
  ye_ += ye;
  psie_ += atan2(Sp, Cp);
  target_k_ += targetk;
  k_samples_++;
}

bool DriveController::GetControl(const DriverConfig &config,
    float throttle_in, float steering_in,
    float *throttle_out, float *steering_out, float dt,
    bool autodrive, int frameno) {

  // compute marginal estimate of trajectory target
  if (k_samples_ > 0) {
    target_k_ /= k_samples_;
    ye_ /= k_samples_;
    psie_ /= k_samples_;
    k_samples_ = 1;
  }

  // okay, let's control for yaw rate!
  // throttle_in controls vmax (w.r.t. the configured value)
  // steering_in controls desired curvature

  // compute target curvature at all times, just for datalogging purposes
  float autok = target_k_;

  // if we're braking or coasting, just control that manually
  if (!autodrive && throttle_in <= 0) {
    *throttle_out = throttle_in;
    // yaw is backwards
    *steering_out =
        clip(-steering_in, STEER_LIMIT_LOW / 127.0, STEER_LIMIT_HIGH / 127.0);
    prev_steer_ = *steering_out;
    prev_throttle_ = *throttle_out;
    prev_v_err_ = 0;
    prev_k_err_ = 0;
    return true;
  }

  // max curvature is 1m radius
  // use a quadratic curve to give finer control near center
  float k = -steering_in * 2 * fabs(steering_in);
  float vk = k;  // curvature to use for velocity calc
  float vmax = throttle_in * config.speed_limit * 0.01;
  if (autodrive) {
    k = autok;
    // use lookahead vk_ to slow down early
    // vk = fmax(fabs(vk_), fabs(k));
    // maybe just use vk_ directly here? then we speed up at corner exit
    vk = fabs(vk_);
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

  float verr = target_v - vr_;
  float kerr = 0;
  if (vr_ > 0.5) {
    kerr = target_k - w_/vr_;
  }

  float BW_w = 0.01 * config.servo_bw;
  float srv_r = 0.01 * config.servo_rate;
  float dkerr = kerr - prev_k_err_;
  float ds = BW_w * (dkerr / srv_r + dt * kerr);
  *steering_out =
      clip(prev_steer_ + ds, STEER_LIMIT_LOW / 127.0, STEER_LIMIT_HIGH / 127.0);
  prev_steer_ = *steering_out;
  prev_k_err_ = kerr;

  float BW_v = 0.01*config.motor_bw;
  float dverr = verr - prev_v_err_;
  float k2 = 0.01 * config.motor_k2;
  float k3 = 0.01 * config.motor_k3;

  // heuristic: subtract magnitude of yaw rate error from throttle control
  float werr = (signbit(target_w) ? -1 : 1) * (target_w - w_) * 0.01 * config.turnin_lift;

  float du = BW_v / (1 - k2 * vr_) * (dverr + dt * k3 * verr - werr * dt);

  *throttle_out = clip(prev_throttle_ + du, -1, 1);
  prev_throttle_ = *throttle_out;
  prev_v_err_ = verr;

  // update state for datalogging
  target_v_ = target_v;
  target_w_ = target_w;
  bw_w_ = BW_w;
  bw_v_ = BW_v;

  return true;
}

int DriveController::SerializedSize() const { return 8 + sizeof(float) * 17; }

int DriveController::Serialize(uint8_t *buf, int buflen) const {
  uint32_t len = SerializedSize();
  assert(buflen >= len);

  memcpy(buf, "CTLs", 4);  // controller state
  memcpy(buf + 4, &len, 4);
  buf += 8;

  memcpy(buf, &x_, 4);
  memcpy(buf + 4, &y_, 4);
  memcpy(buf + 8, &theta_, 4);
  memcpy(buf + 12, &vf_, 4);
  memcpy(buf + 16, &vr_, 4);
  memcpy(buf + 20, &w_, 4);
  memcpy(buf + 24, &prev_steer_, 4);
  memcpy(buf + 28, &prev_throttle_, 4);
  memcpy(buf + 32, &prev_k_err_, 4);

  memcpy(buf + 36, &target_k_, 4);
  memcpy(buf + 40, &target_v_, 4);
  memcpy(buf + 44, &target_w_, 4);
  memcpy(buf + 48, &ye_, 4);
  memcpy(buf + 52, &psie_, 4);
  memcpy(buf + 56, &k_, 4);
  memcpy(buf + 60, &bw_w_, 4);
  memcpy(buf + 64, &bw_v_, 4);

  return len;
}

void DriveController::Dump() const {
  printf("targetkvw %f %f %f v %f k %f",
      target_k_, target_v_, target_w_, vr_, k_);
}
