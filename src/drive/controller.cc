#include "drive/controller.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>

#include "drive/config.h"

using Eigen::Vector3f;

DriveController::DriveController() {
  ResetState();
  if (!V_[0].Init("vf4_1.bin")) {
    perror(
        "*** WARNING: no vf4_1.bin (value function) found, cannot autodrive!");
  }
  if (!V_[1].Init("vf4_2.bin")) {
    perror(
        "*** WARNING: no vf4_2.bin (value function) found, cannot autodrive!");
  }
}

void DriveController::ResetState() {
  vr_ = vf_ = 0;
  w_ = 0;
  prev_throttle_ = 0;
  prev_steer_ = 0;
  ierr_v_ = 0;
  ierr_k_ = 0;
  target_ax_ = 0;
  target_ay_ = 0;
  target_k_ = 0;
  target_v_ = 0;
  vi_ = 0;
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
  ax_ = accel[0];
  ay_ = accel[1];
}

void DriveController::UpdateLocation(const DriverConfig &config,
                                     const float *xytheta) {
  x_ = xytheta[0];
  y_ = xytheta[1];
  theta_ = xytheta[2];

  // ********** HACK HACK HACK **************
  if (vi_ == 1 && y_ < -7.5 && x_ < 7) {
    vi_ = 0;
    printf(" *** switched to map 0 %f %f\n", x_, y_);
  }
  if (vi_ == 0 && x_ > 10.5 && x_ < 12.5 && y_ > -5 && y_ < -3) {
    vi_ = 1;
    printf(" *** switched to map 1 %f %f\n", x_, y_);
  }
}

void _integrate(float theta, float v, float w, float dt, float *x, float *y) {
  float C = cos(theta);
  float S = sin(theta);
  if (abs(w) < 1e-3) {
    *x += dt * v * C;
    *y += dt * v * S;
    return;
  }

  float x0 = dt * w + theta;
  float x1 = v / w;
  *x += x1 * (-sin(theta) + sin(x0));
  *y += x1 * (cos(theta) - cos(x0));
}

void DriveController::Plan(const DriverConfig &config, const int32_t *cardetect,
                           const int32_t *conedetect) {
  const float t0 = theta_ + config.reaction_time * 0.01 * w_;
  const float v0 = clip(vr_, 2, 14);
  float x0 = x_;
  float y0 = y_;
  _integrate(theta_, vr_, w_, config.reaction_time * 0.01, &x0, &y0);

  // best action value, best accel, best curvature
  float cbest = 10e3;
  const float pdt = config.lookahead_time * 0.01;  // predictive delta-t
  const float maxk = fabsf(100.0f / config.servo_rate);
  for (int a = 0; a < kTractionCircleAngles; a++) {
    float phi = a * (2 * M_PI / kTractionCircleAngles);
    float accelx = -config.Ax_limit * 0.01 * cos(phi);
    float accely = config.Ay_limit * 0.01 * sin(phi);
    float k1 = clip(accely / (v0 * v0), -maxk, maxk);
    float w1 = k1 * v0;
    float relang = w1 * pdt;
    float theta1 = t0 + relang;
    float dx = 0;
    float dy = 0;
    // FIXME: min/max speeds hardcoded
    float v1 = clip(v0 + accelx * pdt, 2, 14);
    _integrate(t0, (v0 + v1) / 2, w1, pdt, &dx, &dy);

    float cost = V_[vi_].V(x0 + dx, y0 + dy, theta1, v1);
    // HACK HACK HACK
    // don't go into the other lane!@!@!
    if (vi_ == 0 && x_ > 5.5 && y_ < -3.2 && (y0 + dy) > -3.2) {
      // printf("eliminated drive into wall %f %f -> %f %f\n", x_, y_, x0+dx, y0+dy);
      cost = 100;
    }

    // check whether we hit a cone or a car at this angle
    int iang = (relang * 256 / M_PI) + 128;
    for (int d = -5; d <= 5; d++) {  // no idea what beam width to use here
      cost += cardetect[(iang + d) & 255] * config.car_penalty * 0.01;
      cost += conedetect[(iang + d) & 255] * config.cone_penalty * 0.01;
    }
    // printf("  control (%d %0.3f %0.3f) %f,%f,%f,%f V=%f k=%f v=%f\n", a,
    // accelx,
    //       accely, x0 + dx, y0 + dy, theta1, v1, cost, k1, v1);
    // FIXME: add in obstacle detection here
    target_ks_[a] = k1;
    target_vs_[a] = v1;
    target_Vs_[a] = cost;
    if (cost < cbest) {
      cbest = cost;
      target_k_ = k1;
      target_v_ = v0 + accelx * config.motor_C2 * 0.01f;
      if (accelx < 0) {
        target_v_ = v0 + accelx * config.motor_C1 * 0.01f;
      }
      target_v_ = clip(target_v_, 2, 20);
      target_ax_ = accelx;
      target_ay_ = accely;
    }
  }
  // printf("* best control V=%f k=%f v=%f\n", cbest, target_k_, target_v_);

  /*
  for (int a = 0; a < (1+nangles*2); a++) {
    // compute next x, y, theta
    float k = k0 - (a-nangles)*dk;
    if (fabs(k) < 1e-2)
      k = 1e-2;  // hack: avoid special cases for zero curvature
    float ks = k * s;
    float V = 0;
    float P = 0;
    {
      // jump ahead 30ms before we start planning
      float theta1 = t0 + ks;

      // add up the path cost every 30cm
      for (float st = 0.30; st < s; st += 0.30) {
        // compute relative angle from the car's heading
        float relang = atan2f((1 - cos(k*st))/k, sin(k*st)/k);
        int iang = (relang*256/M_PI) + 128;

        // FIXME(a1k0n): configurable magnitudes here
        int lim = 5 / st;
        for (int d = -lim; d <= lim; d++) {
          P += cardetect[(iang+d) & 255] * config.car_penalty * 0.01;
          P += conedetect[(iang+d) & 255] * config.cone_penalty * 0.01;
        }

        float xt = x0 + (sin(t0 + k*st) - S) / k;
        float yt = y0 + (C - cos(t0 + k*st)) / k;
        P += V_.C(xt, yt) * config.path_penalty * 0.001;
      }
      // and then the final cost-to-go
      float relang = atan2f((1 - cos(ks)) / k, sin(ks) / k);
      int iang = (relang * 256 / M_PI) + 128;
      // printf("k %f relang %f iang %d i %d %f %f\n", k, relang, iang, iang &
      // 255, sin(ks) / k, (1 - cos(ks)) / k);
      V += cardetect[iang & 255] * config.car_penalty * 0.01;
      V += conedetect[iang & 255] * config.cone_penalty * 0.01;
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
  */
}

bool DriveController::GetControl(const DriverConfig &config, float throttle_in,
                                 float steering_in, float *throttle_out,
                                 float *steering_out, float dt, bool autodrive,
                                 int frameno) {
  float srv_off = 0.01 * config.servo_offset;

  float srv_ratio = 100. / (config.servo_rate == 0 ? 100 : config.servo_rate);
  float srv_kI = 0.01 * config.servo_kI;
  float srv_kP = 0.01 * config.servo_kP;

  // if we're braking or coasting, just control that manually
  if (!autodrive && throttle_in <= 0) {
    *throttle_out = throttle_in;
    // yaw is backwards
    *steering_out = clip(
        steering_in * (config.servo_rate < 0 ? 1 : -1) - srv_off * srv_ratio,
        config.servo_min * 0.01, config.servo_max * 0.01);
    prev_steer_ = *steering_out;
    prev_throttle_ = *throttle_out;
    ierr_v_ = 0;
    ierr_k_ = 0;
    return true;
  }

  // pull in the values from Plan() before we mess with them further
  // in case the controller gives infeasible curvatures, clamp them
  // 0.5m smallest turn radius
  float target_k = clip(target_k_, -2, 2);
  float target_a = target_ax_;
  if (vr_ >= config.speed_limit && target_a > 0) {
    target_a = 0;
  }

  float target_v = target_v_;
  if (!autodrive) {
    // manual override:

    // max curvature is servo_rate (unless servo can be driven past 1)
    // use a quadratic curve to give finer control near center
    target_k = -steering_in * 2 * fabs(steering_in);
    target_v = throttle_in * config.speed_limit * 0.01;
    target_a = (target_v - vr_) * config.motor_gain * 0.01;
  } else {
    target_v = clip(target_v_, 0, config.speed_limit * 0.01);
  }

#if 0
  float kmin = config.traction_limit * 0.01 / (vmax*vmax);
  if (fabs(vk) > kmin) {  // any curvature more than this will reduce speed
    target_v = sqrt(config.traction_limit * 0.01 / fabs(vk));
  }
#endif

  // okay, let's control for yaw rate!
  float target_w = target_k * vr_;

  float kerr = 0;
  if (vr_ > 0.2) {
    kerr = target_k - w_ / vr_;
    ierr_k_ = clip(ierr_k_ + dt * srv_kI * kerr, -0.5, 0.5);
  } else {
    ierr_k_ = 0;
  }
  float accelerr = target_ay_ - ay_;
  *steering_out =
      clip((target_k - srv_off + srv_kP * kerr + ierr_k_) * srv_ratio,
           config.servo_min * 0.01, config.servo_max * 0.01);

  prev_steer_ = *steering_out;

#if 1
  float vgain = 0.01 * config.motor_gain;
  float kI = 0.01 * config.motor_kI;
  // boost control gain at high velocities
  // ...or don't, we need to prevent oscillation
  // vgain = clip(vgain / (1 - 0.025*vr_), 0.01, 2.0);
  float verr = target_v - vr_;
  if (prev_throttle_ > -1 && prev_throttle_ < 1) {
    ierr_v_ += verr * dt;
  }
  float u = vgain * (verr + kI * ierr_v_);
#else
  float u0 = config.motor_u0 * 0.01f;
  float u = u0 + (config.motor_kI * 0.01f * target_a +
                  config.motor_C2 * 0.01f * vr_) /
                     (config.motor_C1 * 0.01f - vr_);
  if (target_a < 0 && vr_ > 0) {
    u = -u0 + (config.motor_kI * 0.01f * target_a) / vr_ +
        config.motor_C2 * 0.01f;
  }
  printf("target_a %f vr_ %f u %f\n", target_a, vr_, u);
#endif
  *throttle_out = clip(u, -1, 1);
  prev_throttle_ = *throttle_out;

#if 0  // this is crap
  // heuristic: subtract magnitude of yaw rate error from throttle control
  float werr = fabsf(target_w - w_) * 0.01 * config.turnin_lift;
  *throttle_out = clip(*throttle_out - werr, -1, 1);
#endif

  // update state for datalogging
  target_ax_ = target_a;
  target_v_ = target_v;
  target_w_ = target_w;
  bw_w_ = srv_ratio;
  bw_v_ = config.motor_gain * 0.01f;

  return true;
}

int DriveController::SerializedSize() const {
  return 8 + sizeof(float) * (14 + kTractionCircleAngles * 3);
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
  memcpy(buf, &target_k_, 4);
  buf += 4;
  memcpy(buf, &target_v_, 4);
  buf += 4;
  memcpy(buf, &target_w_, 4);
  buf += 4;
  memcpy(buf, &bw_w_, 4);
  buf += 4;
  memcpy(buf, &bw_v_, 4);
  buf += 4;
  memcpy(buf, &target_ax_, 4);
  buf += 4;
  memcpy(buf, &target_ay_, 4);
  buf += 4;
  for (int a = 0; a < kTractionCircleAngles; a++) {
    memcpy(buf, &target_ks_[a], 4);
    buf += 4;
  }
  for (int a = 0; a < kTractionCircleAngles; a++) {
    memcpy(buf, &target_vs_[a], 4);
    buf += 4;
  }
  for (int a = 0; a < kTractionCircleAngles; a++) {
    memcpy(buf, &target_Vs_[a], 4);
    buf += 4;
  }

  return len;
}

void DriveController::Dump() const { printf("deprecated"); }
