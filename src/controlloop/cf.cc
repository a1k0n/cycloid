#include <stdio.h>
#include <sys/time.h>
#include <iostream>

#include <Eigen/Dense>

#include "hw/car/stm32rs232.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "inih/cpp/INIReader.h"

#include "controlloop/fit.h"
#include "controlloop/pid.h"
#include "controlloop/motorcontrol.h"

const int STEER_LIMIT_LOW = -85;
const int STEER_LIMIT_HIGH = 127;

const float WHEEL_DIAMETER = 0.065;
const float DRIVE_RATIO = 84./25. * 2.1;  // 84t spur, 25t pinion, 2.1 final drive
const float MOTOR_POLES = 3;  // brushless sensor counts per motor revolution
const float V_SCALE = WHEEL_DIAMETER*M_PI / DRIVE_RATIO / MOTOR_POLES;

using Eigen::Vector3f;
using Eigen::Matrix3f;

class CFIR : public JoystickListener, public ControlListener {
 public:
  CFIR(IMU *imu, JoystickInput *js) {
    imu_ = imu;
    js_ = js;
    js_throttle_ = 0;
    js_steering_ = 0;
    exit_ = false;
    recording_ = NULL;
    mode_ = Control;
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
    switch (axis) {
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
    snprintf(fnamebuf, sizeof(fnamebuf), "log-%ld.txt", time(NULL));
    recording_ = fopen(fnamebuf, "w");
    if (recording_ == NULL) {
      perror(fnamebuf);
      return;
    }
    // setlinebuf(recording_);
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

  bool was_learning = false;

  IMU *imu_;
  JoystickInput *js_;

  enum {
    SysId = 0,
    Control = 1
  } mode_;

  int n = 0;

  virtual bool OnControlFrame(CarHW *car, float dt) {
    Vector3f accel, gyro;

    js_->ReadInput(this);
    imu_->ReadIMU(&accel, &gyro);

    float ds, v;
    car->GetWheelMotion(&ds, &v);

    float u_esc = 0;
    float u_steer = 0;
    switch (mode_) {
      case SysId: {
        if (!was_learning) {
          fprintf(stderr, "starting system identification\n");
        }
#if 0
          if (v > 0) {
            // separate brake_id is unsafe for now
            motor_id.AddObservation(v, 1, u_esc, dt);
            steer_id.AddObservation(gyro[2], v, u_steer, dt);
          }
#endif
        // hold PID loops in reset while learning
        // motor_pi.Reset();
        // steer_pi.Reset();
        was_learning = true;
        break;
      }
      case Control: {
        float v_target = 8 * Throttle() / 32767.0;
        if (v_target <= 0) {
          // don't wind up the integrator! just stop the car.
          v_target = 0;
        }
        // turn left -> positive gyro rate, hence negative sign
        float k_target = -Steering() / 32768.0;  // +-1m turning radius

        if (was_learning) {
          /*
          auto km = motor_id.Solve();
          fprintf(stderr, "motor sysid: %f %f %f %f %f\n", km[0], km[1], km[2],
                  km[3], km[4]);
          auto k = steer_id.Solve();
          fprintf(stderr, "steer sysid: %f %f %f %f\n", k[0], k[1], k[2], k[3]);
          */
          was_learning = false;
        }

#if 0
        auto k = motor_id.Solve();
        motor_pi.SetK(1, 0, 0);
        u_esc = motor_pi.Control(v_target - v, dt);
        k = steer_id.Solve();
        steer_pi.SetK(1, 0, 0);
        float kerr = 0;
        if (v > 0.5) {  // must be going at least 0.5 m/s for closed loop yaw
                        // control
          kerr = k_target - gyro[2] / v;
        } else {
          // reset wind-up at low speeds
          steer_pi.Reset();
        }
        u_steer = clipi16(k_target * k[0] + k[3] + steer_pi.Control(kerr, dt),
                          STEER_LIMIT_LOW, STEER_LIMIT_HIGH);
#endif
      } break;
    }

    if (!car->SetControls(n >> 4, u_esc, u_steer)) {
      fprintf(stderr, "SetControls returned false?\n");
    }

    if (recording_) {
      fprintf(recording_, "%0.3f %d %d %f %f %f %f %f %f %f %f %f %f\n",
          dt, Throttle(), Steering(), u_esc, u_steer, ds, v,
          gyro[0], gyro[1], gyro[2],
          accel[0], accel[1], accel[2]);
    }

    n++;
    return !exit_;
  }
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
  JoystickInput js;
  INIReader ini("cycloid.ini");
  I2C i2c;
  IMU *imu = IMU::GetI2CIMU(i2c, ini);
  CarHW *car = CarHW::GetCar(&i2c, ini);

  if (!i2c.Open()) {
    fprintf(stderr, "need to enable i2c in raspi-config, probably\n");
    return 1;
  }

  if (!js.Open(ini)) {
    return 1;
  }
  if (!car->Init()) {
    return 1;
  }
  if (!imu->Init()) {
    return 1;
  }

  // for fwd, use model dv/dt = [k1 k2 k3] . [u v 1]
  // prior [1 0.1 -1]

  // for brake, use different [k1 k2 k3] but same model

  uint16_t wpos, wperiod;
  uint16_t last_wpos = 0;

  CFIR ir(imu, &js);

  setlinebuf(stdout);

  car->RunMainLoop(&ir);
}

