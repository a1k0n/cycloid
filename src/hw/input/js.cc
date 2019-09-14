#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <string>

#include "hw/input/js.h"
#include "inih/cpp/INIReader.h"

JoystickInput::JoystickInput() {
  fd_ = -1;
  buttons_ = 0;
  memset(axes_, 0, sizeof(axes_));
  buttonmap_ = NULL;
  axismap_ = NULL;
}

JoystickInput::~JoystickInput() {
  if (fd_ != -1) {
    close(fd_);
  }
}

bool JoystickInput::Open(const INIReader &ini) {
  std::string jtype = ini.GetString("joystick", "type", "");

  // N.B. there's no axis map; the axes are mostly the same on the two
  // controllers we support, but that may have to change

  if (jtype == "wiiupro") {  // Wii U Pro controller via Bluetooth
    buttonmap_ = "BAXYLRlr-+H,.";
    axismap_ = "01234567";
  } else if (jtype == "f710") {  // Logitech F710 via USB dongle
    // a few buttons and axes are swapped w.r.t. the Wii U controller here
    buttonmap_ = "BAYXLR-+H,.??";
    axismap_ = "01324567";
  }

  std::string buttonmap = ini.GetString("joystick", "buttonmap", "");
  if (buttonmap != "") {
    buttonmap_ = buttonmap.c_str();
  }

  std::string axismap = ini.GetString("joystick", "axismap", "");
  if (axismap != "") {
    axismap_ = axismap.c_str();
  }

  if (!buttonmap_ || !axismap_) {
    fprintf(stderr,
            "Please specify cycloid.ini [joystick]:\n"
            "  either axismap=0123... buttonmap=BAXY... or "
            "type=wiiupro/f710/etc\n");
    return false;
  }

  std::string jsdevice = ini.GetString("joystick", "device", "/dev/input/js0");
  fd_ = open(jsdevice.c_str(), O_RDONLY);
  if (fd_ == -1) {
    perror(jsdevice.c_str());
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
      if (oldvalue != value) {
        if (number < 13) {
          if (value) {
            receiver->OnButtonPress(buttonmap_[number & 15]);
          } else {
            receiver->OnButtonRelease(buttonmap_[number & 15]);
          }
        } else if (number < 17) {
          static const char *dpadmap = "UDLR";
          if (value) {
            receiver->OnDPadPress(dpadmap[number - 13]);
          } else {
            receiver->OnDPadRelease(dpadmap[number - 13]);
          }
        }
      }
      if (value) {
        buttons_ |= (1 << number);
      } else {
        buttons_ &= ~(1 << number);
      }
    } else if (type == 0x02) {  // axis
      // logitech F710 axes 6, 7 are DPad
      if (number == 6) {
        if (value < 0) {
          receiver->OnDPadPress('L');
        } else if (value > 0) {
          receiver->OnDPadPress('R');
        }
      } else if (number == 7) {
        if (value < 0) {
          receiver->OnDPadPress('U');
        } else if (value > 0) {
          receiver->OnDPadPress('D');
        }
      } else if (number < 8) {
        receiver->OnAxisMove(axismap_[number] - '0', value);
      }
    }
  }
  return newvalue;
}
