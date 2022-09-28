#include "drive/controller.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>

#include "drive/config.h"

using Eigen::Vector3f;

DriveController::DriveController() {
  ResetState();
  if (!pi_.Load("pinet.bin")) {
    perror(" *** WARNING: failed to load pinet.bin\n");
  }
  if (!track_.LoadTrack("track.txt")) {
    perror(" *** WARNING: failed to load track.txt\n");
  }
}

void DriveController::ResetState() {
  vr_ = vf_ = 0;
  w_ = 0;
  prev_throttle_ = 0;
  prev_steer_ = 0;
  ix_ = -1;
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
  if (ix_ == -1) {
    ix_ = track_.ClosestIdx(x_, y_);
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

#if 0
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
#endif

bool DriveController::GetControl(const DriverConfig &config, float throttle_in,
                                 float steering_in, float *throttle_out,
                                 float *steering_out, float dt, bool autodrive,
                                 int frameno) {
  float srv_off = 0.01 * config.servo_offset;
  float srv_min = 0.01 * config.servo_min;
  float srv_max = 0.01 * config.servo_max;

  float throttle_max = 0.01 * config.throttle_cap;
  if (prev_throttle_ + config.throttle_slew * 0.01 * dt < throttle_max) {
    throttle_max = prev_throttle_ + config.throttle_slew * 0.01 * dt;
  }
  float throttle_min = prev_throttle_ - config.throttle_slew * 0.01 * dt;
  if (throttle_min < -1) {
    throttle_min = -1;
  }

  // rev limiter
  if (vr_ >= config.speed_limit * 0.01) {
    throttle_max = 0.95*prev_throttle_;
  }

  // update track index and get our local coordinate frame
  // (need to do this even if we aren't autodriving, just to sync localization)
  float xl, yl, cl, sl;
  if (ix_ != -1) {
    // we could dead-reckon x,y,theta by integrating over v, w; for now let's not
    track_.LocalState(x_, y_, theta_, &ix_, &xl, &yl, &cl, &sl);
  }

  if (!autodrive || ix_ == -1) {
    *throttle_out = clip(throttle_in, throttle_min, throttle_max);
    *steering_out = clip(steering_in + srv_off, srv_min, srv_max);
  } else {
    // autodrive
    float u_throttle = 0, u_steer = 0;
    pi_.Action(ix_, vr_ * config.pi_v_scale * 0.01f, w_, xl, yl, cl, sl,
              config.pi_thr_scale * 0.01f, config.pi_brake_scale * 0.01f,
              config.pi_steer_scale * 0.01f,
              &u_throttle, &u_steer);
    // fprintf(stderr, "                        ix %d vr %+0.2f w %+0.2f xl %+0.2f yl %+0.2f cl %+0.2f sl %+0.2f u_throttle %+0.2f (%+0.2f,%+0.2f) u_steer %+0.2f\r",
    //         ix_, vr_, w_, xl, yl, cl, sl, u_throttle, throttle_min, throttle_max, u_steer);
    // fflush(stderr);
    *throttle_out = clip(u_throttle + config.throttle_bias * 0.01f, throttle_min, throttle_max);
    *steering_out = clip(u_steer + srv_off, srv_min, srv_max);
  }

  prev_steer_ = *steering_out;
  prev_throttle_ = *throttle_out;

  return true;
}

int DriveController::SerializedSize() const {
  return 8 + sizeof(float) * 7;
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

  return len;
}

void DriveController::Dump() const { printf("deprecated"); }
