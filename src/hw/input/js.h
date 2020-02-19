// joystick input
// control car with a moga bluetooth controller

#ifndef HW_INPUT_JS_H_
#define HW_INPUT_JS_H_

#include <stdint.h>

#include "hw/input/input.h"

class INIReader;

class JoystickInput {
 public:
  JoystickInput();
  ~JoystickInput();

  bool Open(const INIReader &ini);

  // Read latest car input from joystick
  bool ReadInput(JoystickListener *receiver);

  int GetFileDescriptor() { return fd_; }

 private:
  int fd_;

  uint32_t buttons_;
  int16_t axes_[8];
  const char *buttonmap_;
  const char *axismap_;
};

#endif  // HW_INPUT_JS_H_
