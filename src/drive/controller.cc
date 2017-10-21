#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

#include "drive/controller.h"
#include "drive/imgproc.h"

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::VectorXf;


DriveController::DriveController() {
  ResetState();
}

void DriveController::ResetState() {
  ekf_.Reset();
  localiz_.ResetUnknown();
  firstframe_ = true;
}

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

//extern float __centerline_R_hack;

void DriveController::UpdateCamera(const DriverConfig &config,
    int32_t *reprojected, uint8_t *annotated) {
  Vector3f B;
  Matrix4f Rk = Matrix4f::Zero();
  float yc;

  if (!imgproc::TophatFilter(config.yellow_thresh, reprojected, &B, &yc, &Rk,
        annotated)) {
    return;
  }

  // ok, we've obtained our linear fit B and our measurement covariance Rk
  // now do the sensor fusion step

  float a = B[0], b = B[1], c = B[2];
  ekf_.UpdateCenterline(a, b, c, yc, Rk);

  const Eigen::VectorXf &x = ekf_.GetState();
  float ye = x[2];
  float psie = x[3];
  float kappa = x[4];
  float ds = yc * cos(psie) + 0;  // lookahead distance; make config?
  localiz_.Update(ds, kappa, 0.01 / ekf_.GetCovariance()(4, 4));
  // localiz_.Update(ds, kappa, 0.01 / __centerline_R_hack);

  float roff = localiz_.GetRacelineOffset();
  float rphi = localiz_.GetRacelineAngle();
  float rtan = -tan(rphi - psie);
  // update annotated display with centerline state, raceline curve
  // mx = pixel_scale_m * (px + ux0)  -> x, y in meters given x, y in pixels
  // my = pixel_scale_m * (py + uy0)
  // px = mx / pixel_scale_m - ux0  -> vice versa
  // py = my / pixel_scale_m - uy0
  for (int py = 0; py < imgproc::uysiz; py++) {
    float my = imgproc::pixel_scale_m * (py + imgproc::uy0);
    float mx = a*my*my + b*my + c;
    int px = mx * (1.0/imgproc::pixel_scale_m) - imgproc::ux0;
    if (px >= 0 && px < imgproc::uxsiz) {
      annotated[(px + py*imgproc::uxsiz)*3] = 128;
      annotated[(px + py*imgproc::uxsiz)*3 + 1] = 0;
      annotated[(px + py*imgproc::uxsiz)*3 + 2] = 0;
    }

    // get the raceline position here
    mx = roff - ye + rtan * my;  // y offset
    px = mx * (1.0/imgproc::pixel_scale_m) - imgproc::ux0;
    if (px >= 0 && px < imgproc::uxsiz) {
      annotated[(px + py*imgproc::uxsiz)*3] = 128;
      annotated[(px + py*imgproc::uxsiz)*3 + 1] = 255;
      annotated[(px + py*imgproc::uxsiz)*3 + 2] = 255;
    }
  }
}

void DriveController::UpdateState(const DriverConfig &config,
    int32_t *reprojected, float throttle_in, float steering_in,
    const Vector3f &accel, const Vector3f &gyro,
    uint8_t servo_pos, const uint16_t *wheel_encoders, float dt,
    uint8_t *annotated) {
  Eigen::VectorXf &x_ = ekf_.GetState();
  if (isinf(x_[0]) || isnan(x_[0])) {
    fprintf(stderr, "WARNING: kalman filter diverged to inf/NaN! resetting!\n");
    ResetState();
    // exit(1);
    return;
  }

  if (firstframe_) {
    memcpy(last_encoders_, wheel_encoders, 4*sizeof(uint16_t));
    firstframe_ = false;
  }

  ekf_.Predict(dt, throttle_in, steering_in);
  std::cout << "x after predict " << x_.transpose() << std::endl;
  localiz_.Predict(x_, ekf_.GetCovariance(), dt);

  UpdateCamera(config, reprojected, annotated);

  ekf_.UpdateIMU(gyro[2]);
  std::cout << "x after IMU (" << gyro[2] << ")" << x_.transpose() << std::endl;

  // hack: force psi_e forward-facing
  if (x_[3] > M_PI/2) {
    x_[3] -= M_PI;
  } else if (x_[3] < -M_PI/2) {
    x_[3] += M_PI;
  }

  // read / update servo & encoders
  // use the average of the two rear encoders as we're most interested in the
  // motor speed
  // but we could use all four to get turning radius, etc.
  // since the encoders are just 16-bit counters which wrap frequently, we only
  // track the difference in counts between updates.
  printf("encoders were: %05d %05d %05d %05d\n"
         "      are now: %05d %05d %05d %05d\n",
      last_encoders_[0], last_encoders_[1], last_encoders_[2], last_encoders_[3],
      wheel_encoders[0], wheel_encoders[1], wheel_encoders[2], wheel_encoders[3]);

  // average ds among wheel encoders which are actually moving
  float ds = 0, nds = 0;
  for (int i = 2; i < 4; i++) {  // only use rear encoders
    if (wheel_encoders[i] != last_encoders_[i]) {
      ds += (uint16_t) (wheel_encoders[i] - last_encoders_[i]);
      nds += 1;
    }
  }
  memcpy(last_encoders_, wheel_encoders, 4*sizeof(uint16_t));
  // and do an EKF update if the wheels are moving.
  if (nds > 0) {
    ekf_.UpdateEncoders(ds/(nds * dt), servo_pos);
    std::cout << "x after encoders (" << ds/dt << ") " << x_.transpose() << std::endl;
  } else {
    ekf_.UpdateEncoders(0, servo_pos);
    std::cout << "x after encoders (" << ds/dt << ") " << x_.transpose() << std::endl;
  }

  std::cout << "P " << ekf_.GetCovariance().diagonal().transpose() << std::endl;
}

/*
 * def motorcontrol(k1, k2, k3, v, a):
    u_V = v > 0 and a > -k3 * v
    u_DC = (a + k3 * v) / (u_V * k1 - v * k2)
    dc = u_DC * (2*u_V - 1)
    if a > 0 and dc < 0.13:
        return 0.13  # account for dead zone
    return dc
*/

static float MotorControl_model(const DriverConfig &config,
    float v_target,
    float k1, float k2, float k3, float v) {
  float kP = (v_target >= v ? config.motor_kP : config.brake_kP) * 0.01;
  float accel = clip(kP * (v_target - v), -7, 7);
  // voltage (1 or 0)
  // v > 0 ????
  float V = accel > -k3 * v ? 1 : 0;
  // duty cycle
  float DC = clip(
      (accel + k3 * v) / (V*k1 - k2*v),
      0, config.max_throttle * 0.01);
  const float deadzone_min = config.motor_offset * 0.01;
  if (accel > 0 && DC < deadzone_min) {
    DC = deadzone_min;
  }
  return V == 1 ? DC : -DC;
}

static float MotorControl_model(float accel,
    float k1, float k2, float k3, float k4,
    float v) {
  if (v < 0.2) {
    k4 += k3;  // add static friction onto dynamic friction
  }
  float a_thresh = -k4;
  // voltage (1 or 0)
  float V = accel > a_thresh ? 1 : 0;
  // duty cycle
  float DC = clip(
      (accel + k4) / (V*k1 - k2*v),
      0, 1);
  return V == 1 ? DC : -DC;
}

#if 1
static float MotorControlPID(const DriverConfig &config,
    float v_target, float v) {
  static float last_err = 0;
  static float ierr = 0;
  float err = v_target - v;

  ierr = ierr * 0.95 + err;
  float derr = err - last_err;
  last_err = err;

  printf("pid err %f derr %f ierr %f\n", err, derr, ierr);
  if (err > 0) {
    return clip(config.motor_offset + err * config.motor_kP
        - derr * config.motor_kD + ierr * config.motor_kI,
        -100, config.max_throttle) * 0.01;
  } else {
    return clip(err * config.brake_kP, -100, config.max_throttle) * 0.01;
  }
}
#endif

bool DriveController::GetControl(const DriverConfig &config,
    float *throttle_out, float *steering_out, float dt) {
  const Eigen::VectorXf &x_ = ekf_.GetState();
  float v = x_[0];
  float delta = x_[1];
  float y_e = x_[2];
  float psi_e = x_[3];
  float kappa = x_[4];
  float ml_1 = x_[5];
  float ml_2 = x_[6];
  float ml_3 = x_[7];
  float ml_4 = x_[8];
  float srv_a = x_[9];
  float srv_b = x_[10];
  float srv_r = x_[11];

  float k1 = exp(ml_1), k2 = exp(ml_2), k3 = exp(ml_3);

  float vmax = config.speed_limit * 0.01;

  // race line following
#if 0
  // this doesn't work well enough, i'm scrapping the idea
  float lane_offset = localiz_.GetRacelineOffset();
  float psi_offset = localiz_.GetRacelineAngle();
  // float lane_offset = 0;
  // float psi_offset = 0;
  kappa = localiz_.GetRacelineCurvature();
  vmax = fmin(vmax, localiz_.GetRacelineVelocity());
#else
  float lane_offset = clip(config.lane_offset
      + kappa * config.lane_offset_per_k, -100.0, 100.0) * 0.01;
  float psi_offset = 0;
#endif

  float cpsi = cos(psi_e - psi_offset),
        spsi = sin(psi_e - psi_offset);
  // float dx = cpsi / (1.0 - kappa*(y_e - lane_offset));
  float dx = cpsi / (1.0 - kappa*y_e);

  // Alain Micaelli, Claude Samson. Trajectory tracking for unicycle-type and
  // two-steering-wheels mobile robots. [Research Report] RR-2097, INRIA. 1993.
  // <inria-00074575>
  const float kpy = 0.01 * config.steering_kpy;
  const float kvy = 0.01 * config.steering_kvy;

  // it's a little backwards though because our steering is reversed w.r.t.
  // curvature
  float k_target = dx * (-(y_e - lane_offset) * dx * kpy*cpsi
      - spsi*(-kappa*spsi - kvy*cpsi) + kappa);

  *steering_out = clip((k_target - srv_b) / srv_a, -1, 1);
  if (*steering_out == -1 || *steering_out == 1) {
    // steering is clamped, so we may need to further limit speed
    float w_target = v * k_target;
    float k_limit = srv_a * (*steering_out) + srv_b;
    vmax = fmin(vmax, w_target / k_limit);
  }

  const float tlimit = 0.01 * config.traction_limit;
  float v_target = fmin(vmax, sqrtf(tlimit / fabs(k_target)));
#if 0
  float a_target = clip(v_target - v, BRAKE_LIMIT, ACCEL_LIMIT) / dt;
  if (a_target > 0) {  // accelerate more gently than braking
    a_target /= 4;
  } else {
    a_target /= 2;  // ???
  }
#else
  *throttle_out = MotorControlPID(config, v_target, v);
//#else
//  *throttle_out = MotorControl_model(config, v_target, k1, k2, k3, v);
#endif

  printf("steer_target %0.2f delta %0.2f v_target %0.2f v %0.2f "
      "lateral_a %0.2f/%0.2f v %0.2f y %0.2f psi %0.2f\n",
      k_target, delta, v_target, v, v*v*delta, 0.01 * config.traction_limit,
      v, y_e, psi_e);

  printf("  throttle %f steer %f\n", *throttle_out, *steering_out);
  return true;
}

