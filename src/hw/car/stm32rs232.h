#ifndef HW_CAR_STM32RS232_H_
#define HW_CAR_STM32RS232_H_

#include <stdint.h>
#include "hw/car/car.h"

// rs232-connected stm32f030 running a program to write to servo / ESC, read
// from brushless motor sensor

class STM32HatSerial: public CarHW {
 public:
  explicit STM32HatSerial(const INIReader &ini);

  virtual bool Init();
  virtual bool SetControls(unsigned led, float throttle, float steering);
  virtual bool GetWheelMotion(float *ds, float *v);
  virtual void RunMainLoop(ControlListener *cb);

 private:
  // this is blocking until the next frame sync, returns motor encoder position
  // and speed
  bool AwaitSync(uint16_t *encoder_pos, uint16_t *encoder_dt);

  int fd_;
  bool sync_;
  const char *device_;
  float meters_per_tick_;
  float ds_, v_;
};

#endif  // HW_CAR_STM32RS232_H_

