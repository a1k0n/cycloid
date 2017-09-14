#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "hw/input/js.h"

// assumes MOGA 2 controller, doesn't bother to read axis labels or anything

JoystickInput::JoystickInput() {
  fd_ = -1;
  buttons_ = 0;
  memset(axes_, 0, sizeof(axes_));
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

bool JoystickInput::ReadInput(InputReceiver *receiver) {
  bool newvalue = false;

  while (fd_ != -1) {
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
    newvalue = true;
    if (type == 0x01) {  // button
      value = value ? 1 : 0;
      int16_t oldvalue = (buttons_ >> number) & 1;
      static const char *buttonmap = "ABXYLRS789abcdef";
      if (oldvalue != value) {
        if (value) {
          receiver->OnButtonPress(buttonmap[number & 15]);
        } else {
          receiver->OnButtonRelease(buttonmap[number & 15]);
        }
      }
      if (value) {
        buttons_ |= (1 << number);
      } else {
        buttons_ &= ~(1 << number);
      }
    } else if (type == 0x02) {  // axis
      if (number < 6) {
        receiver->OnAxisMove(number, value);
      } else if (number < 8) {  // axes 6 and 7 are d-pad
        static const char *negdpad = "LU";
        static const char *posdpad = "RD";
        if (value != axes_[number]) {
          if (axes_[number] < -16384) {  // release existing negative direction
            receiver->OnDPadRelease(negdpad[number - 6]);
          } else if (axes_[number] > 16384) {  // release existing positive direction
            receiver->OnDPadRelease(posdpad[number - 6]);
          }

          axes_[number] = value;

          if (value < -16384) {  // press existing negative direction
            receiver->OnDPadPress(negdpad[number - 6]);
          } else if (value > 16384) {  // press existing positive direction
            receiver->OnDPadPress(posdpad[number - 6]);
          }
        }
      }
    }
  }
  return newvalue;
}
