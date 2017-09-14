#include "car/teensy.h"

static const int TEENSY_ADDRESS = 118;

static const int ADDR_CONTROL = 0x00;
// 00 - bit 0: teensy LED
//    - bit 4: disable RC passthrough (unsupported)
static const int ADDR_PWM = 0x01;
// 01 - PWM channel 1
// 02 - PWM channel 2
// 03 - reserved
// 04 - reserved
static const int ADDR_RC = 0x05;
// 05 - RC input channel 1  (unsupported yet)
// 06 - RC input channel 2  (unsupported yet)
static const int ADDR_SRV = 0x07;
// 07 - servo position voltage
static const int ADDR_ENCODER_COUNT = 0x08;
// 08 - encoder #1 count (low)
// 09 - encoder #1 count (high)
// 0a - encoder #2 count (low)
// 0b - encoder #2 count (high)
// 0c - encoder #3 count (low)
// 0d - encoder #3 count (high)
// 0e - encoder #4 count (low)
// 0f - encoder #4 count (high)
static const int ADDR_ENCODER_PERIOD = 0x10;
// 10 - encoder #1 period (low)
// 11 - encoder #1 period (high)
// 12 - encoder #2 period (low)
// 13 - encoder #2 period (high)
// 14 - encoder #3 period (low)
// 15 - encoder #3 period (high)
// 16 - encoder #4 period (low)
// 17 - encoder #4 period (high)
static const int NUM_ADDRS = 0x18;

Teensy::Teensy(const I2C &i2cbus) : i2c_(i2cbus) {}

bool Teensy::Init() {
  return i2c_.Write(TEENSY_ADDRESS, 0x00, 0);
}

bool Teensy::SetControls(uint8_t led, int8_t esc, int8_t servo) {
  uint8_t buf[3] = {led, esc, servo};
  return i2c_.Write(TEENSY_ADDRESS, 0, 3, buf);
}

bool Teensy::GetFeedback(uint8_t *servo, uint16_t *encoder_pos,
    uint16_t *encoder_dt) {
  // Because the Arduino Wire API is super crappy,
  // we have to read the entire block
  uint8_t buf[NUM_ADDRS];
  if (!i2c_.Read(TEENSY_ADDRESS, 0, NUM_ADDRS, buf)) {
    return false;
  }

  *servo = buf[ADDR_SRV];
  for (int i = 0; i < 4; i++) {
    encoder_pos[i] = buf[ADDR_ENCODER_COUNT + 2*i]
      + (buf[ADDR_ENCODER_COUNT + 1 + 2*i] << 8);
  }

  for (int i = 0; i < 4; i++) {
    encoder_dt[i] = buf[ADDR_ENCODER_PERIOD + 2*i]
      + (buf[ADDR_ENCODER_PERIOD + 1 + 2*i] << 8);
  }

  return true;
}
