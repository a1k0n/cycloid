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
  ierr_v_ = 0;
  ierr_w_ = 0;
}

static inline float clip(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

void DriveController::UpdateState(const DriverConfig &config,
    const Vector3f &accel, const Vector3f &gyro,
    uint8_t servo_pos, const uint16_t *wheel_period, float dt) {

  // FIXME: hardcoded servo calibraiton
  // delta_ = (servo_pos - 126.5) / 121.3;
  delta_ = 0;

#if 0
  // update front/rear velocity estimate through crude filter
  if (ACTIVE_ENCODERS == 4) {
    vf_ *= (1 - V_ALPHA);
    vf_ += V_ALPHA * V_SCALE * 0.5*(wheel_delta[0] + wheel_delta[1])/dt;
    vr_ *= (1 - V_ALPHA);
    vr_ += V_ALPHA * V_SCALE * 0.5*(wheel_delta[2] + wheel_delta[3])/dt;
  } else {
    vf_ *= (1 - V_ALPHA);
    float sum = 0;
    for (int i = 0; i < ACTIVE_ENCODERS; i++) {
      sum += wheel_delta[i];
    }
    vf_ += V_ALPHA * V_SCALE * sum * (1.0 / ACTIVE_ENCODERS) / dt;
    vr_ = vf_;  // assume vr==vf, AWD
  }
#else
  // with new STM32 hat firmware, this isn't necessary and we have a better
  // wheel speed estimate
  if (wheel_period[0] == 0) {
    vf_ = vr_ = 0;
  } else {
    vf_ = vr_ = V_SCALE * 1e6 / wheel_period[0];
  }
#endif

  w_ = gyro[2];
}

void DriveController::UpdateLocation(const DriverConfig &config,
    float x, float y, float theta) {
  x_ = x;
  y_ = y;
  theta_ = theta; // NOTE: theta is meaningless here; I shouldn't be tracking it
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
    *steering_out = clip(-steering_in, STEER_LIMIT_LOW/127.0, STEER_LIMIT_HIGH/127.0);
    ierr_w_ = 0;  // also reset integrators
    ierr_v_ = 0;
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
    // maybe just use vk_ directly here? then we speed up at corner exit
    vk = fmax(fabs(vk_), fabs(k));
    vmax = config.speed_limit * 0.01;
  }

  float kmin = config.traction_limit * 0.01 / (vmax*vmax);

  float target_v = vmax;
  if (fabs(vk) > kmin) {  // any curvature more than this will reduce speed
    target_v = sqrt(config.traction_limit * 0.01 / fabs(vk));
    float atarget = config.accel_limit * 0.01;

    // maintain an optimal slip ratio with 0 lateral velocity
    // by adjusting speed until vf = vr*cos(delta) - w*Lf*sin(delta)
    // vr = (vf + w*Lf*sin(delta)) / cos(delta)
#if 0
    float vr_slip_target = (vf_ + atarget + w_*GEOM_LF*sin(delta_)) /
        cos(delta_);
    if (vr_slip_target < target_v && vr_slip_target > 1.0) {
      // printf("[%d] using slip target %f (vf=%f vr=%f)\n",
      //     frameno, vr_slip_target, vf_, vr_);
      target_v = vr_slip_target;
    }
#else
    //target_v = clip(target_v, target_v + atarget, vmax);
#endif
  }

  // use current velocity to determine target yaw rate
  // this yaw rate should be achievable with our tires given the slip rate
  // limit above
  float target_k = k;
  float target_w = k*vr_;

  float err_v = vr_ - target_v;
  float kerr = 0;
  if (vr_ > 0.5) {
    kerr = target_k - w_/vr_;
  } else {
    ierr_w_ = 0;
  }

  float BW_w = 2*M_PI*0.01*config.yaw_bw;

  // *steering_out = clip(-BW_w/target_v * (ierr_w_ + err_w / BW_SRV), -1, 1);
  // why did i divide by target_v? that seems crazy and in practice it goes nuts
  // at low speeds unless BW_w is tiny.
  // *steering_out = clip(-BW_w * (ierr_w_ + err_w / BW_SRV),
  //    STEER_LIMIT_LOW/127.0, STEER_LIMIT_HIGH/127.0);

  *steering_out = SERVO_DIRECTION*clip(target_k*SRV_A + SRV_D + BW_w * (ierr_w_*SRV_A + kerr*SRV_B),
     STEER_LIMIT_LOW/127.0, STEER_LIMIT_HIGH/127.0);

  float BW_v = 2*M_PI*0.01*config.motor_bw;
  float Kp = BW_v / (M_K1 - M_K2*vr_);
  float Ki = M_K3;
  *throttle_out = clip(-Kp*(err_v + Ki*ierr_v_) + M_OFFSET, 0, 1);
  if (*throttle_out == 0 && vr_ > 0) {  // handle braking
    // alternate control law
    float Kp2 = BW_v / (-M_K2*vr_);
    *throttle_out = clip(Kp2*(err_v + Ki*ierr_v_ - M_OFFSET), -1, 0);
  }

  // don't wind-up at control limits
  if ((*throttle_out > -1 && *throttle_out < 1) ||
      (err_v > 0 && ierr_v_ < 0) || (err_v < 0 && ierr_v_ > 0)) {
    ierr_v_ += dt*err_v;
  }

  if ((*steering_out > -1 && *steering_out < 1) ||
      (kerr > 0 && ierr_w_ < 0) || (kerr < 0 && ierr_w_ > 0)) {
    ierr_w_ += dt*kerr;
  }

  // update state for datalogging
  target_v_ = target_v;
  target_w_ = target_w;
  bw_w_ = BW_w;
  bw_v_ = BW_v;

  return true;
}

int DriveController::SerializedSize() const {
  return sizeof(float)*17;
}

int DriveController::Serialize(uint8_t *buf, int buflen) const {
  assert(buflen >= 68);

  memcpy(buf, &x_, 4);
  memcpy(buf+4, &y_, 4);
  memcpy(buf+8, &theta_, 4);
  memcpy(buf+12, &vf_, 4);
  memcpy(buf+16, &vr_, 4);
  memcpy(buf+20, &w_, 4);
  memcpy(buf+24, &ierr_v_, 4);
  memcpy(buf+28, &ierr_w_, 4);
  memcpy(buf+32, &delta_, 4);

  memcpy(buf+36, &target_k_, 4);
  memcpy(buf+40, &target_v_, 4);
  memcpy(buf+44, &target_w_, 4);
  memcpy(buf+48, &ye_, 4);
  memcpy(buf+52, &psie_, 4);
  memcpy(buf+56, &k_, 4);
  memcpy(buf+60, &bw_w_, 4);
  memcpy(buf+64, &bw_v_, 4);

  return 68;
}

void DriveController::Dump() const {
  printf("targetkvw %f %f %f v %f k %f windup %f %f",
      target_k_, target_v_, target_w_, vr_, k_, ierr_v_, ierr_w_);
}
