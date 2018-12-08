#ifndef HW_CAR_STM32I2C_H_
#define HW_CAR_STM32I2C_H_

#include "hw/gpio/i2c.h"

// i2c-connected stm32f030 running a program to write to servo / ESC, read from
// brushless motor sensor

class STM32Hat {
 public:
  explicit STM32Hat(const I2C &i2cbus);

  bool Init();

  bool SetControls(uint8_t led, int8_t esc, int8_t servo);
  bool GetFeedback(uint16_t *encoder_pos, uint16_t *encoder_dt);

 private:
  const I2C &i2c_;
};

#endif  // HW_CAR_STM32I2C_H_

