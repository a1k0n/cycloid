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

  void Reset() { last_err = 0; ierr = 0; }

 private:
  float Ki, Kp, Kd;
  float last_err;
  float ierr;
};

#endif  // CONTROLLOOP_PID_H_
