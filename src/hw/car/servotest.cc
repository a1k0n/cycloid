#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

// #include "car/pca9685.h"
#include "car/teensy.h"
#include "gpio/i2c.h"

int main() {
  I2C i2c;

  if (!i2c.Open()) {
    return 1;
  }

#if 0
  PCA9685 pca(i2c);

  pca.Init(100);  // 100Hz output

  // pwm period is 10ms
  // servo range is 1..2ms, so 409.6 .. 819.2
  // center both output channels
  pca.SetPWM(0, 614.4);
  pca.SetPWM(1, 614.4);

  int i = 0;
  for (;;) {
    pca.SetPWM(0, 614.4 + 200*sin(i * 0.01));
    i += 1;
    usleep(10000);
  }
#else

  Teensy teensy(i2c);
  teensy.Init();

  int i = 0;
  for (i = 0; i < 400; i++) {
    uint8_t servo;
    uint16_t encoders[4];
    uint16_t encoderpw[4];
    teensy.SetControls(i & 32 ? 1 : 0, 0, 60*sin(i* 0.01));
    teensy.GetFeedback(&servo, encoders, encoderpw);
    printf("servo %d encoders %05d %05d %05d %05d\r",
        servo, encoders[0], encoders[1], encoders[2], encoders[3]);
    fflush(stdout);
    usleep(10000);
  }
  teensy.SetControls(0, 0, 0);
#endif
}
