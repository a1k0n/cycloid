#ifndef HW_CAR_PCA9685_H_
#define HW_CAR_PCA9685_H_

#include "hw/gpio/i2c.h"

// deprecated, doesn't conform to CarHW spec and no plans to add it now
class PCA9685 {
 public:
  explicit PCA9685(const I2C &i2cbus);

  bool Init(float pwm_freq_hz);

  // PCA9685 has 12 bits of resolution, so 0..4095
  // relative to the PWM frequency set above
  void SetPWM(uint8_t channel, uint16_t duty);
 private:
  const I2C &i2c_;
};

#endif  // HW_CAR_PCA9685_H_
