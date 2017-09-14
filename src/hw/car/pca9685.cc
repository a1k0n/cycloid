#include "car/pca9685.h"
#include "gpio/i2c.h"

static const int PCA9685_ADDRESS = 0x40;
// registers:
static const int MODE1      = 0x00;
static const int MODE2      = 0x01;
static const int PRESCALE   = 0xFE;
static const int LED0_ON_L  = 0x06;
static const int LED0_ON_H  = 0x07;
static const int LED0_OFF_L = 0x08;
static const int LED0_OFF_H = 0x09;


// Bits:
static const int RESTART = 0x80;
static const int SLEEP   = 0x10;
static const int ALLCALL = 0x01;
static const int INVRT   = 0x10;
static const int OUTDRV  = 0x04;


PCA9685::PCA9685(const I2C &i2cbus) : i2c_(i2cbus) {}

bool PCA9685::Init(float pwm_freq_hz) {
  if (!i2c_.Write(PCA9685_ADDRESS, 0x06, 0)) {  // reset
    return false;
  }

  float prescaler = 25e6 / 4096 / pwm_freq_hz - 1;
  i2c_.Write(PCA9685_ADDRESS, PRESCALE, (uint8_t) (prescaler + 0.5));
  i2c_.Write(PCA9685_ADDRESS, MODE2, OUTDRV);
  i2c_.Write(PCA9685_ADDRESS, MODE1, ALLCALL);

  return true;
}

void PCA9685::SetPWM(uint8_t channel, uint16_t duty) {
  i2c_.Write(PCA9685_ADDRESS, LED0_OFF_L + 4*channel, duty & 0xff);
  i2c_.Write(PCA9685_ADDRESS, LED0_OFF_H + 4*channel, duty >> 8);
}
