#include "drive/driver.h"
#include "hw/car/car.h"
#include "hw/gpio/i2c.h"
#include "inih/cpp/INIReader.h"

class MotorLearn : public ControlCallback {
  int frameno;
  const int maxframes = 100 * 10;  // ramp over 10 seconds

 public:
  MotorLearn() { frameno = 0; }

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

int main(int argc, char *argv[]) {
  I2C i2c;
  CarHW *carhw;
  MotorLearn ml;

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

  carhw = CarHW::GetCar(&i2c, ini);
  if (!carhw || !carhw->Init()) {
    fprintf(stderr, "failed to init car hardware\n");
    return 1;
  }

  carhw->RunMainLoop(&ml);
}
