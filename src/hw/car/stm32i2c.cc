#include <sys/time.h>
#include <unistd.h>

#include "hw/car/stm32i2c.h"
#include "hw/car/car.h"
#include "inih/cpp/INIReader.h"

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
// 07 - radio ch1 period (low)
// 08 - radio ch1 period (high)
// 09 - radio ch2 period (low)
// 0a - radio ch2 period (high)
static const int NUM_ADDRS = 0x0b;

STM32Hat::STM32Hat(I2C *i2cbus, const INIReader &ini) : i2c_(i2cbus) {
  if (!ini.HasValue("car", "meters_per_wheeltick")) {
    fprintf(stderr,
            "STM32HatSerial: please specify [car] meters_per_wheeltick\n"
            " -- cannot use wheel encoder without it\n");
  }
  meters_per_tick_ = ini.GetReal("car", "meters_per_wheeltick", 0);
  ds_ = v_ = 0;
  radio_in_[0] = 0;
  radio_in_[1] = 0;
}

bool STM32Hat::Init() {
  return i2c_->Write(STM32HAT_ADDRESS, 0x00, 0);
}

bool STM32Hat::SetControls(unsigned led, float throttle, float steering) {
  if (throttle < -1) throttle = -1;
  else if (throttle > 1) throttle = 1;
  if (steering < -1) steering = -1;
  else if (steering > 1) steering = 1;

  int8_t esc = 127.0*throttle;
  int8_t servo = 127.0*steering;
  uint8_t buf[3] = {(uint8_t)led, (uint8_t)esc, (uint8_t)servo};
  return i2c_->Write(STM32HAT_ADDRESS, 0, 3, buf);
}

bool STM32Hat::GetWheelMotion(float *ds, float *v) {
  if (meters_per_tick_ == 0) {
    return false;
  }
  *ds = ds_;
  *v = v_;
  ds_ = 0;
  return true;
}


void STM32Hat::RunMainLoop(ControlListener *cb) {
  const int N = NUM_ADDRS - ADDR_ENCODER_COUNT;
  uint8_t buf[N];
  uint16_t last_wpos;
  timeval last_t;

  gettimeofday(&last_t, NULL);
  for (;;) {
    // sync to next 100Hz frame
    timeval t;
    gettimeofday(&t, NULL);
    useconds_t udt =
        (t.tv_sec - last_t.tv_sec) * 1000000 + (t.tv_usec - last_t.tv_usec);
    if (udt < 10000) usleep(10000 - udt);

    if (!i2c_->Read(STM32HAT_ADDRESS, ADDR_ENCODER_COUNT, N, buf)) {
      return;
    }
    gettimeofday(&t, NULL);

    uint16_t wpos = buf[0] + (buf[1] << 8);
    uint16_t wheeldt = buf[2] + (buf[3] << 8);

    radio_in_[0] = buf[4] + (buf[5] << 8);
    radio_in_[1] = buf[6] + (buf[7] << 8);

    uint16_t wheel_delta = wpos - last_wpos;
    last_wpos = wpos;
    ds_ += meters_per_tick_ * wheel_delta;
    if (wheeldt > meters_per_tick_ * 1e6 / 50.0) {
      // occasionally the firmware outputs a ridiculously small but nonzero
      // wheel period, so restrict to reasonable values (< 50m/s max)
      v_ = meters_per_tick_ * 1e6 / wheeldt;
    } else if (wheeldt == 0) {
      v_ = 0;
    }
    float dt = t.tv_sec - last_t.tv_sec + (t.tv_usec - last_t.tv_usec) * 1e-6;
    if (!cb->OnControlFrame(this, dt)) {
      break;
    }
    last_t = t;
  }
}

int STM32Hat::GetRadioInput(float *channelbuf, int maxch) {
  if (maxch < 2) {
    return 0;
  }
  if (radio_in_[0] < 800 || radio_in_[1] < 800) {
    return 0;
  }
  channelbuf[0] = (radio_in_[0] - 1500) / 500.0f;
  channelbuf[1] = (radio_in_[1] - 1500) / 500.0f;
  return 2;
}
