#ifndef CONTROLLOOP_PID_H_
#define CONTROLLOOP_PID_H_

// utility class to handle PID integration / differentiation

class PIDLoop {
 public:
  PIDLoop() { Reset(); }

  void SetK(float ki, float kp, float kd) {
    Ki = ki;
    Kp = kp;
    Kd = kd;
  }

  float Control(float err, float dt) {
    float derr = (err - last_err) / dt;
    ierr += err * dt;
    last_err = err;

    return Kp*err + Ki*ierr + Kd*derr;
  }

  void Reset() {
    last_err = 0;
    ierr = 0;
  }

 private:
  float Ki, Kp, Kd;
  float last_err;
  float ierr;
};

// proportional-integral controller with antiwindup
// actually a PD controller on the control deltas
// assumes controls are limited to -1..1
class PILoop {
 public:
  PILoop() { Reset(); }

  void SetK(float ki, float kp) {
    Ki = ki;
    Kp = kp;
  }

  void Reset() {
    last_err = 0;
    last_u = 0;
  }

  float Control(float err, float dt) {
    float derr = (err - last_err);
    last_err = err;

    // integrate u and clip
    float u = last_u + Kp*derr + Ki*err*dt;
    if (u > 1) {
      u = 1;
    } else if (u < -1) {
      u = -1;
    }
    last_u = u;
    return u;
  }

 private:
  float Ki, Kp;
  float last_err;
  float last_u;
};

#endif  // CONTROLLOOP_PID_H_
