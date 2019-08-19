#ifndef HW_CAR_CAR_H_
#define HW_CAR_CAR_H_

class INIReader;

class CarHW;

class ControlCallback {
 public:
  // Callback returns false to exit main loop
  virtual bool OnControlFrame(CarHW *car, float dt) = 0;
};

class CarHW {
 public:
  virtual bool Init() = 0;

  // set speed controller and servo to desired setting (-1..1) and LEDs, if any,
  // according to the bitmask in LEDs
  virtual bool SetControls(unsigned LEDs, float throttle, float steering) = 0;

  // returns true if we have wheel encoders, and sets ds to meters travelled
  // since last frame and v to wheel velocity
  virtual bool GetWheelMotion(float *ds, float *v) = 0;

  // Run control loop at configured frequency (usually 100Hz) until callback
  // returns false.
  virtual void RunMainLoop(ControlCallback *cb) = 0;

  static CarHW *GetCar(const INIReader &ini);
};

#endif  // HW_CAR_CAR_H_
