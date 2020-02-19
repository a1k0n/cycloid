#ifndef HW_CAR_STM32I2C_H_
#define HW_CAR_STM32I2C_H_

#include "hw/car/car.h"
#include "hw/gpio/i2c.h"

// i2c-connected stm32f030 running a program to write to servo / ESC, read from
// brushless motor sensor

// deprecated, doesn't conform to CarHW spec and no plans to add it now
class STM32Hat: public CarHW {
 public:
  STM32Hat(I2C *i2cbus, const INIReader &ini);

  virtual bool Init();
  virtual bool SetControls(unsigned led, float throttle, float steering);
  virtual bool GetWheelMotion(float *ds, float *v);
  virtual void RunMainLoop(ControlCallback *cb);

  bool SetControls(uint8_t led, int8_t esc, int8_t servo);
  bool GetFeedback(uint16_t *encoder_pos, uint16_t *encoder_dt);

 private:
  I2C *i2c_;
  float meters_per_tick_;
  float ds_, v_;
};

#endif  // HW_CAR_STM32I2C_H_

