#include "hw/car/stm32i2c.h"

static const int STM32HAT_ADDRESS = 0x75;

static const int ADDR_CONTROL = 0x00;
// 00 - bit 0: LED1 (red)
//    - bit 1: LED2 (green)
static const int ADDR_PWM = 0x01;
// 01 - PWM channel 1
// 02 - PWM channel 2
static const int ADDR_ENCODER_COUNT = 0x03;
// 03 - motor tick count (low)
// 04 - motor tick count (high)
static const int ADDR_ENCODER_PERIOD = 0x05;
// 05 - motor tick period (low)
// 06 - motor tick period (high)
// 07 - motor tick period (high)
// 08 - motor tick period (high)
static const int NUM_ADDRS = 0x07;

STM32Hat::STM32Hat(const I2C &i2cbus) : i2c_(i2cbus) {}

bool STM32Hat::Init() {
  return i2c_.Write(STM32HAT_ADDRESS, 0x00, 0);
}

bool STM32Hat::SetControls(uint8_t led, int8_t esc, int8_t servo) {
  uint8_t buf[3] = {led, (uint8_t)esc, (uint8_t)servo};
  return i2c_.Write(STM32HAT_ADDRESS, 0, 3, buf);
}

bool STM32Hat::GetFeedback(uint16_t *encoder_pos, uint16_t *encoder_dt) {
  const int N = NUM_ADDRS - ADDR_ENCODER_COUNT;
  uint8_t buf[N];
  if (!i2c_.Read(STM32HAT_ADDRESS, ADDR_ENCODER_COUNT, N, buf)) {
    return false;
  }

  *encoder_pos = buf[0] + (buf[1] << 8);
  *encoder_dt  = buf[2] + (buf[3] << 8);

  return true;
}
