#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>

#include "coneslam/localize.h"
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
  if (!V_.Init()) {
    perror("*** WARNING: no vf.bin (value function) found, cannot autodrive!");
  }
}

void DriveController::ResetState() {
  vr_ = vf_ = 0;
  w_ = 0;
  prev_throttle_ = 0;
  prev_steer_ = 0;
  prev_v_err_ = 0;
  ierr_k_ = 0;
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
  } else if (wheel_period[0] > V_SCALE * 1e6 / 30.0) {
    // occasionally the firmware outputs a ridiculously small but nonzero wheel
    // period, so restrict to reasonable values (< 30m/s max)
    vf_ = vr_ = V_SCALE * 1e6 / wheel_period[0];
  }

  w_ = gyro[2];
}

void DriveController::UpdateLocation(const DriverConfig &config,
                                     const coneslam::Localizer *l) {
  coneslam::Particle meanp;
  l->GetLocationEstimate(&meanp);
  x_ = meanp.x;
  y_ = meanp.y;
}

void DriveController::Plan(const DriverConfig &config,
                           const coneslam::Particle *ps, int np) {
  // TODO(asloane): consider more feasible maneuvers
  // curvature can swing about 0.3 1/m from wherever it is now in 10ms
  float k0 = 0;
  float kmax = 1.3;
  float dk = config.lookahead_krate * 0.01 / 3;
  if (vr_ > 0.3) {
    k0 = clip(w_ / vr_, -kmax + 3 * dk, kmax - 3 * dk);
  }

  timeval tv0, tv1;
  gettimeofday(&tv0, NULL);
  float s = config.lookahead_dist * 0.01;
  for (int a = 0; a < 7; a++) {
    // compute next x, y, theta
    float k = k0 - (a-3)*dk;
    float ks = k * s;
    float V = 0;
    float P = 0;
    for (int i = 0; i < np; i++) {
      float x0 = ps[i].x, y0 = ps[i].y, t0 = ps[i].theta;
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
  // gettimeofday(&tv1, NULL);
  // printf(" control %ld.%06lds\n", tv1.tv_sec - tv0.tv_sec,
  //       tv1.tv_usec - tv0.tv_usec);

  // compute target curvature at all times, just for datalogging purposes
  float bestV = target_k_Vs_[0];
  target_k_ = target_ks_[0];
  for (int a = 1; a < 7; a++) {
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
    prev_v_err_ = 0;
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

  float verr = target_v - vr_;
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

  float BW_v = 0.01 * config.motor_bw;
  float dverr = verr - prev_v_err_;
  float k2 = 0.01 * config.motor_k2;

  // heuristic: subtract magnitude of yaw rate error from throttle control
  float werr = fabsf(target_w - w_) * 0.01 * config.turnin_lift;

  float du = BW_v * (dverr + k2 * prev_throttle_ * verr * dt);

  *throttle_out = clip(prev_throttle_ + du, -1, 1);
  prev_throttle_ = *throttle_out;
  // hack in yaw error here
  *throttle_out = clip(*throttle_out - werr, -1, 1);
  prev_v_err_ = verr;

  // update state for datalogging
  target_v_ = target_v;
  target_w_ = target_w;
  bw_w_ = BW_w;
  bw_v_ = BW_v;

  return true;
}

int DriveController::SerializedSize() const {
  return 8 + sizeof(float) * (12 + 2 * 7);
}

int DriveController::Serialize(uint8_t *buf, int buflen) const {
  uint32_t len = SerializedSize();
  assert(buflen >= len);

  memcpy(buf, "CTL2", 4);  // controller state
  memcpy(buf + 4, &len, 4);
  buf += 8;

  memcpy(buf, &x_, 4);
  buf += 4;
  memcpy(buf, &y_, 4);
  buf += 4;
  memcpy(buf, &vf_, 4);
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
  for (int a = 0; a < 7; a++) {
    memcpy(buf, &target_ks_[a], 4);
    buf += 4;
  }
  for (int a = 0; a < 7; a++) {
    memcpy(buf, &target_k_Vs_[a], 4);
    buf += 4;
  }

  return len;
}

void DriveController::Dump() const {
  printf("deprecated");
}
