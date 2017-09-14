// joystick input
// control car with a moga bluetooth controller

#ifndef HW_INPUT_JS_H_
#define HW_INPUT_JS_H_

#include <stdint.h>

class JoystickInput {
 public:
  JoystickInput();
  ~JoystickInput();

  bool Open();

  // Read latest car input from joystick
  bool ReadInput(int *throttle, int *steering, uint16_t *buttons);

  int GetFileDescriptor() { return fd_; }

 private:
  int fd_;

  int throttle_, steering_;
  int steertrim_;
  uint16_t buttons_;
};

#endif  // HW_INPUT_JS_H_
