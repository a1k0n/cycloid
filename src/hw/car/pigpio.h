#ifndef HW_CAR_PIGPIO_H_
#define HW_CAR_PIGPIO_H_

#include <stdint.h>
#include "hw/car/car.h"

// rs232-connected stm32f030 running a program to write to servo / ESC, read
// from brushless motor sensor

class PiGPIOCar: public CarHW {
 public:
  explicit PiGPIOCar(const INIReader &ini);

  virtual bool Init();
  virtual bool SetControls(unsigned led, float throttle, float steering);
  virtual bool GetWheelMotion(float *ds, float *v);
  virtual void RunMainLoop(ControlCallback *cb);

 private:
  int escpin_, servopin_;
  int pwmfreq_;
  bool fwdonly_;
};

#endif  // HW_CAR_PIGPIO_H_
