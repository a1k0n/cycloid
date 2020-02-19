#include "drive/driver.h"
#include "hw/car/car.h"
#include "hw/imu/imu.h"
#include "hw/gpio/i2c.h"
#include "inih/cpp/INIReader.h"

class ControlRamp : public ControlListener {
  int frameno;
  const int maxframes = 100 * 10;  // ramp over 10 seconds

 public:
  ControlRamp() { frameno = 0; }

  bool OnControlFrame(CarHW *car, float dt) override {
    float ds, v;
    car->GetWheelMotion(&ds, &v);
    printf("%f %f\n", frameno * 1.0f / maxframes, v);
    frameno++;
    if (frameno == maxframes) {
      car->SetControls(0, 0, 0);
      return false;
    }
    car->SetControls(0, frameno * 1.0f / maxframes, 0);
    return true;
  }
};

class AccelTest : public ControlListener {
  int frameno;
  IMU *imu_;
  const int maxframes = 300;
  float u;

 public:
  explicit AccelTest(IMU *imu) {
    frameno = 0;
    imu_ = imu;
    u = 0;
  }

  bool OnControlFrame(CarHW *car, float dt) override {
    Eigen::Vector3f accel, gyro;
    float ds, v;
    car->GetWheelMotion(&ds, &v);
    imu_->ReadIMU(&accel, &gyro);
    printf("%f %f %f %f %f %f\n", frameno / 100.0f, u, v, accel[0], accel[1],
           accel[2]);

    frameno++;
    if (frameno > 150) {
      u = -1.0;
    } else if (frameno > 70) {
      u = 0;
    } else if (frameno > 2) {
      u = 1.0;
    }
    if (frameno == maxframes) {
      car->SetControls(0, 0, 0);
      return false;
    }
    car->SetControls(0, u, 0);  // maybe zero out steering?
    return true;
  }
};

int main(int argc, char *argv[]) {
  I2C i2c;
  CarHW *carhw;
  IMU *imu;

  INIReader ini("cycloid.ini");
  {
    int inierr = ini.ParseError();
    if (inierr != 0) {
      if (inierr == -1) {
        fprintf(stderr, "please create cycloid.ini!\n");
      } else {
        fprintf(stderr, "error loading cycloid.ini on line %d\n", inierr);
      }
      return 1;
    }
  }

  if (!i2c.Open()) {
    fprintf(stderr, "need to enable i2c in raspi-config, probably\n");
    return 1;
  }


  imu = IMU::GetI2CIMU(i2c, ini);
  if (!imu || !imu->Init()) {
    fprintf(stderr, "unable to connect to IMU; aborting\n");
    return 1;
  }

  carhw = CarHW::GetCar(&i2c, ini);
  if (!carhw || !carhw->Init()) {
    fprintf(stderr, "failed to init car hardware\n");
    return 1;
  }

  // ControlRamp cr;
  // carhw->RunMainLoop(&cr);

  AccelTest acceltest(imu);
  carhw->RunMainLoop(&acceltest);
}
