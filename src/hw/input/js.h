// joystick input
// control car with a moga bluetooth controller

#ifndef HW_INPUT_JS_H_
#define HW_INPUT_JS_H_

#include <stdint.h>

#include "./input.h"

class INIFile;

class JoystickInput {
 public:
  explicit JoystickInput(const INIFile &ini);
  ~JoystickInput();

  bool Open();

  // Read latest car input from joystick
  bool ReadInput(InputReceiver *receiver);

  int GetFileDescriptor() { return fd_; }

 private:
  int fd_;

  uint32_t buttons_;
  int16_t axes_[8];
};

#endif  // HW_INPUT_JS_H_
