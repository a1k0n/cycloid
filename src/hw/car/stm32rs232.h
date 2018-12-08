#ifndef HW_CAR_STM32RS232_H_
#define HW_CAR_STM32RS232_H_

#include <stdint.h>

// rs232-connected stm32f030 running a program to write to servo / ESC, read
// from brushless motor sensor

class STM32HatSerial {
 public:
  STM32HatSerial();

  bool Init();

  bool SetControls(uint8_t led, int8_t esc, int8_t servo);

  // this is blocking until the next frame sync, returns motor encoder position
  // and speed
  bool AwaitSync(uint16_t *encoder_pos, uint16_t *encoder_dt);

 private:
  int fd_;
  bool sync_;
};

#endif  // HW_CAR_STM32RS232_H_

