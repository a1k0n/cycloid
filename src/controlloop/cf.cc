#include <stdio.h>
#include <sys/time.h>
#include <iostream>

#include <Eigen/Dense>

#include "hw/car/stm32rs232.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"

#include "controlloop/fit.h"
#include "controlloop/pid.h"

const int STEER_LIMIT_LOW = -85;
const int STEER_LIMIT_HIGH = 127;

const float WHEEL_DIAMETER = 0.0666;
const float DRIVE_RATIO = 84./25. * 2.1;  // 84t spur, 25t pinion, 2.1 final drive
const float MOTOR_POLES = 3;  // brushless sensor counts per motor revolution
const float V_SCALE = WHEEL_DIAMETER*M_PI / DRIVE_RATIO / MOTOR_POLES;

const float MOTOR_BW = 0.5 * 6.28;  // motor control loop bandwidth
const float YAW_BW = 0.5 * 6.28;  // yaw control loop bandwidth

using Eigen::Vector3f;
using Eigen::Matrix3f;

static float clipf(float x, float min, float max) {
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}

static int16_t clipi16(int16_t x, int16_t min, int16_t max) {
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}

class CFIR : public InputReceiver {
 public:
  CFIR() {
    js_throttle_ = 0;
    js_steering_ = 0;
    exit_ = false;
    recording_ = NULL;
    mode_ = SysId;
  }
  virtual ~CFIR() {}

  virtual void OnButtonPress(char button) {
    switch (button) {
      case '+':  // plus button; start recording
        StartRecording();
        break;
      case '-':  // minus button; stop recording
        StopRecording();
        break;
      case 'L':
        mode_ = SysId;
        break;
      case 'R':
        mode_ = Control;
        break;
    }
  }

  virtual void OnAxisMove(int axis, int16_t value) {
    switch(axis) {
      case 1:  // left stick y axis
        js_throttle_ = -value;
        break;
      case 2:  // right stick x axis
        js_steering_ = value;
        break;
    }
  }

  void StartRecording() {
    if (recording_) {
      StopRecording();
    }
    char fnamebuf[256];
    sprintf(fnamebuf, "log-%ld.txt", time(NULL));
    recording_ = fopen(fnamebuf, "w");
    if (recording_ == NULL) {
      perror(fnamebuf);
      return;
    }
    setlinebuf(recording_);
    fprintf(recording_, "# dt u_esc u_steer dw wperiod gx gy gz ax ay az\n");
    fprintf(stderr, "recording %s\n", fnamebuf);
  }

  void StopRecording() {
    if (recording_ == NULL) {
      return;
    }
    fclose(recording_);
    recording_ = NULL;
    fprintf(stderr, "stopped recording\n");
  }

  int16_t Throttle() { return js_throttle_; }
  int16_t Steering() { return js_steering_; }

  int16_t js_throttle_;
  int16_t js_steering_;
  bool exit_;
  FILE *recording_;

  enum {
    SysId = 0,
    Control = 1
  } mode_;
};

/*
void controltest() {
  float target_v = (n & 256) ? 200 : 400;

  // observe effect of previous input and update model
  float v = 0;
  if (wperiod != 0) {
    v = 1.0e6 / wperiod;
  }

  // loop bandwidth
  const float Wc = 0.5 * 6.28;

  // initial PID constants just to kick-start it; they will quickly be overwritten
  float Kp = 0.05;
  float Ki = 10*0.05;
  float Kd = 0;

  if (v != 0 && last_v != 0) {
    float dvdt = (v - last_v) / dt;
    float d2vdt2 = (dvdt - last_dvdt) / dt;
    //Vector3f X(last_u[0], last_u[0]*v, v, 1);
    Vector3f X(v, dvdt, d2vdt2);
    //XTY *= 0.97;
    //XTX *= 0.97;

    XTY += X * last_u[0];
    XTX += X * X.transpose();

    Eigen::LDLT<Matrix3f> ALDLT = XTX.ldlt();
    Vector3f k = ALDLT.solve(XTY);
    Ki = Wc * k[0];
    Kp = Wc * k[1];
    Kd = Wc * k[2];
    printf("c1=%f c2=%f c3=%f Ki=%f Kp=%f Kd=%f\n", k[0], k[1], k[2], Ki, Kp, Kd);

    last_dvdt = dvdt;
  }

#if 0
  const float Kp = 5;
  float target_vdot = Kp * (target_v - v);
  if (k[0] < 1) k[0] = 1;
  if (k[1] > 0) k[1] = 0;
  if (k[2] > 0) k[2] = 0;
  if (k[3] > 0) k[3] = 0;
  float uu = (target_vdot - k[2]*v - k[3]) / (k[0] + k[1]*v);
#if 0
  float uu = 0;
  if (n & 32) {
    uu += 32 - (n & 31);
  } else {
    uu += n & 31;
  }
#endif
#endif

  float err = target_v - v;
  float derr = (err - last_err) / dt;
  ierr += err * dt;
  last_err = err;
  float uu = Kp*err + Ki*ierr + Kd*derr;

  int8_t u = clipf(uu, -127, 127);
  // if (target_vdot > 0 && u < 10) u = 10;
  if (!car.SetControls(n >> 4, u, 0)) {
    fprintf(stderr, "SetControls returned false?\n");
  }
  // printf("%0.3f wpos:%d wperiod:%d k:%f %f %f %f v:%f tv:%f vdot:%f uu:%f u:%d lu:%d\n", dt, wpos, wperiod, k[0], k[1], k[2], k[3], v, target_v, target_vdot, uu, u, last_u[0]);
  printf("%0.3f wpos:%d wperiod:%d v:%f tv:%f uu:%f ierr:%f u:%d lu:%d\n", dt, wpos, wperiod, v, target_v, uu, ierr, u, last_u[0]);

  n++;
  // printf("   %d %d %d %d\n", last_u[0], last_u[1], last_u[2], last_u[3]);
  for (int i = 0; i < ESC_DELAY-1; i++) {
    last_u[i] = last_u[i+1];
  }
  last_v = v;
#endif
}
*/

int main(int argc, char *argv[]) {
  STM32HatSerial car;
  JoystickInput js;
  I2C i2c;
  IMU imu(i2c);

  if (!i2c.Open()) {
    fprintf(stderr, "need to enable i2c in raspi-config, probably\n");
    return 1;
  }

  if (!js.Open()) {
    return 1;
  }
  if (!car.Init()) {
    return 1;
  }
  if (!imu.Init()) {
    return 1;
  }

  // for fwd, use model dv/dt = [k1 k2 k3] . [u v 1]
  // prior [1 0.1 -1]

  // for brake, use different [k1 k2 k3] but same model

  uint16_t wpos, wperiod;
  uint16_t last_wpos = 0;

  Eigen::Vector3f accel, gyro;

  int n = 0;
  car.AwaitSync(&last_wpos, &wperiod);
  timeval t0;
  gettimeofday(&t0, NULL);

  CFIR ir;

  SysIdentifier motor_id, steer_id;
  PIDLoop motor_pid, steer_pid;

  bool was_learning = true;
  setlinebuf(stdout);
  while (!ir.exit_ && car.AwaitSync(&wpos, &wperiod)) {
    timeval t1;
    gettimeofday(&t1, NULL);
    float dt = (t1.tv_sec + t1.tv_usec * 1e-6)
      - (t0.tv_sec + t0.tv_usec * 1e-6);

    js.ReadInput(&ir);
    {
      float temp;
      imu.ReadIMU(&accel, &gyro, &temp);
    }

    uint16_t dw = wpos - last_wpos;
    float v = 0;
    if (wperiod != 0) {
      v = V_SCALE * 1e6 / wperiod;
    } else {
      v = V_SCALE * dw / dt;
    }
    // special case: if we stopped really quickly, then wperiod might still be
    // nonzero but dw is zero; this is only valid if wperiod is >10000us
    if (dw == 0 && wperiod < 10000) {
      v = 0;
    }

    int8_t u_esc = 0;
    int8_t u_steer = 0;
    switch (ir.mode_) {
      case ir.SysId:
        {
          u_esc = clipi16(ir.Throttle() >> 8, -127, 127);
          u_steer = clipi16(ir.Steering() >> 8,
              STEER_LIMIT_LOW, STEER_LIMIT_HIGH);
          if (v > 0) {
            // separate brake_id is unsafe for now
            motor_id.AddObservation(v, 1, u_esc, dt);
            steer_id.AddObservation(gyro[2], v, u_steer, dt);
          }
        }
        // hold PID loops in reset while learning
        motor_pid.Reset();
        steer_pid.Reset();
        was_learning = true;
        break;
      case ir.Control:
        {
          float v_target = 8 * ir.Throttle() / 32767.0;
          if (v_target <= 0) {
            // don't wind up the integrator! just stop the car.
            v_target = 0;
          }
          // turn left -> positive gyro rate, hence negative sign
          float k_target = -ir.Steering() / 32768.0;  // +-1m turning radius

          if (was_learning) {
            Eigen::Vector4f k = motor_id.Solve();
            fprintf(stderr, "motor sysid: %f %f %f %f\n", k[0], k[1], k[2], k[3]);
            k = steer_id.Solve();
            fprintf(stderr, "steer sysid: %f %f %f %f\n", k[0], k[1], k[2], k[3]);
            was_learning = false;
          }

          Eigen::Vector4f k = motor_id.Solve();
#if 0
          motor_pid.SetK(MOTOR_BW*k[0], MOTOR_BW*k[1], MOTOR_BW*k[2]);
          u_esc = clipi16(k[3] + motor_pid.Control(v_target - v, dt), -100, 127);
#else
          motor_pid.SetK(0, MOTOR_BW*k[1], MOTOR_BW*k[2]);
          u_esc = clipi16(v_target*k[0] + k[3] + motor_pid.Control(v_target - v, dt), -32, 127);
#endif
          k = steer_id.Solve();
          steer_pid.SetK(YAW_BW*k[0], YAW_BW*k[1], YAW_BW*k[2]);
          float kerr = 0;
          if (v > 0.5) {  // must be going at least 0.5 m/s for closed loop yaw control
            kerr = k_target - gyro[2]/v;
          } else {
            // reset wind-up at low speeds
            steer_pid.Reset();
          }
          if (v == 0) {
            motor_pid.Reset();
          }
          u_steer = clipi16(k_target*k[0] + k[3] + steer_pid.Control(kerr, dt),
              STEER_LIMIT_LOW, STEER_LIMIT_HIGH);
        }
        break;
    }

    if (!car.SetControls(n >> 4, u_esc, u_steer)) {
      fprintf(stderr, "SetControls returned false?\n");
    }

    if (ir.recording_) {
      fprintf(ir.recording_, "%0.3f %d %d %d %d %d %d %f %f %f %f %f %f\n",
          dt, ir.Throttle(), ir.Steering(), u_esc, u_steer, dw, wperiod,
          gyro[0], gyro[1], gyro[2],
          accel[0], accel[1], accel[2]);
    }

    t0 = t1;
    last_wpos = wpos;
    n++;
  }
}

