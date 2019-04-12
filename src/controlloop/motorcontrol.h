#ifndef CONTROLLOOP_MOTORCONTROL_H_
#define CONTROLLOOP_MOTORCONTROL_H_

#include "controlloop/fit.h"

static float clipf(float x, float min, float max) {
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}

// A five-parameter electric motor + ESC model:
// V === normalized voltage (1 for go, 0 for brake)
// dc === duty cycle (the magnitude of the input)

// we have a term for dc and for dc^2 as we assume the ESC response is
// nonlinear, so we do a quadratic fit

//         ---motor torque----   -----back EMF------   drag
// dv/dt = k1*V*dc + k2*V*dc^2 + k3*v*dc + k4*v*dc^2 + k5*v

// we then do loop-shaping to make the closed loop response a 1-pole lowpass
// filter of the target velocity with bandwidth wc.

class SelfTuningMotorControl {
 public:
  SelfTuningMotorControl(float scale=1) { Reset(scale); }

  void Reset(float scale=1) {
    sysid_.Reset(scale);
    ResetControl();
  }

  void ResetControl() {
    ierr_ = 0;
    last_u_ = 0;
    pdc_ = 0;  // previous duty cycle, moving average
  }

  int8_t Control(float target_v, float cur_v, float wc, float dt) {
    float err = target_v - cur_v;
    sysid_.AddObservation(cur_v, last_u_, dt);
    auto k = sysid_.Solve();

    ierr_ += dt*err*(k[2]*pdc_ + k[3]*pdc_*pdc_ + k[4])/(k[0] + k[1]*pdc_);
    float u = clipf(wc*(err/(k[0] + k[1]*pdc_) - ierr_), -127, 127);
    // we low-pass filter our current duty cycle
    // in order to prevent oscillations
    pdc_ = 0.875*pdc_ + 0.125*fabs(u);
    last_u_ = u;
    return (int8_t) u;
  }

  void AddObservation(float v, float u, float dt) {
    sysid_.AddObservation(v, u, dt);
  }

  MotorSysId sysid_;

 private:
  float ierr_;
  float last_u_;
  float pdc_;
};

#endif  // CONTROLLOOP_MOTORCONTROL_H_
