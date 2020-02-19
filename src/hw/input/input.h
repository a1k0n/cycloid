#ifndef INPUT_H_
#define INPUT_H_

class JoystickListener {
 public:
  virtual void OnDPadPress(char direction) {}
  virtual void OnDPadRelease(char direction) {}

  virtual void OnButtonPress(char button) {}
  virtual void OnButtonRelease(char button) {}

  virtual void OnAxisMove(int axis, int16_t value) {}
};

#endif  // INPUT_H_
