#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "input/js.h"

// assumes MOGA 2 controller, doesn't bother to read axis labels or anything

JoystickInput::JoystickInput() {
  throttle_ = 0;
  steering_ = 0;
  steertrim_ = 0;
  buttons_ = 0;
  fd_ = -1;
}

JoystickInput::~JoystickInput() {
  if (fd_ != -1) {
    close(fd_);
  }
}

bool JoystickInput::Open() {
  fd_ = open("/dev/input/js0", O_RDONLY);
  if (fd_ == -1) {
    perror("/dev/input/js0");
    return false;
  }

  int opt = O_NONBLOCK;
  if (fcntl(fd_, F_SETFL, opt) < 0) {
    perror("F_SETFL O_NONBLOCK");
  }

  return true;
}

bool JoystickInput::ReadInput(int *throttle, int *steering, uint16_t *buttons) {
  bool newvalue = false;

  while (fd_ != -1) {
    // set saved values
    *throttle = throttle_;
    *steering = steering_ + steertrim_;
    *buttons = buttons_;

    uint8_t buf[8];
    int n = read(fd_, buf, 8);
    if (n < 0) {
      if (errno == EAGAIN) {
        break;
      }
      perror("JoystickInput");
      close(fd_);
      fd_ = -1;
      break;
    }
    if (n != 8) {
      fprintf(stderr, "JoystickInput: short read (%d), closing\n", n);
      close(fd_);
      fd_ = -1;
      break;
    }
    int16_t value = buf[4] + (buf[5] << 8);
    uint8_t type = buf[6];
    uint8_t number = buf[7];
    if (type == 0x01) {  // button
      switch (number) {
#if 0
        case 0:  // A, steer trim left
          steertrim_ -= 100;
          newvalue = true;
          break;
        case 1:  // B, steer trim right
          steertrim_ += 100;
          newvalue = true;
          break;
#endif
        default:
          if (value) {
            buttons_ |= (1 << number);
          } else {
            buttons_ &= ~(1 << number);
          }
          newvalue = true;
          break;
      }
    } else if (type == 0x02) {  // axis
      switch (number) {
        case 1:  // left stick y axis, inverted (up is more throttle)
          throttle_ = -value;
          newvalue = true;
          break;
        case 2:  // right stick x axis
          steering_ = value;
          newvalue = true;
          break;
      }
    }
  }
  return newvalue;
}
