#include <sys/time.h>
#include <unistd.h>

#include "hw/car/car.h"
#include "hw/car/pigpio.h"
#include "inih/cpp/INIReader.h"
#include "pigpio/pigpio.h"

PiGPIOCar::PiGPIOCar(const INIReader &ini) {
  escpin_ = ini.GetInteger("car", "escpin", 12);
  servopin_ = ini.GetInteger("car", "servopin", 13);
  pwmfreq_ = ini.GetInteger("car", "pwmfreq", 100);
  // "fwdonly" means the ESC starts at zero throttle at a 1ms pulse, and only
  // goes forward up to a max 2ms pulse, like an ESC for a quad or airplane.
  // false means 1.5ms is neutral, more than that is forward, and less than that is brake.
  fwdonly_ = ini.GetBoolean("car", "fwdonly", false);
}

bool PiGPIOCar::Init() {
  if (gpioInitialise() == PI_INIT_FAILED) {
    return false;
  }
  SetControls(0, 0, 0);
}

bool PiGPIOCar::SetControls(unsigned led, float throttle, float steering) {
  // s = (duty/PI_HW_PWM_RANGE)/freq
  // freq*s*PI_HW_PWM_RANGE = duty
  // s = input*.0005 + .0015
  if (fwdonly_) {  // turn 0..1 to -1..1 in the fwdonly case
    throttle = throttle*2 - 1;
    if (throttle < -1) throttle = -1;
  }

  gpioHardwarePWM(escpin_, pwmfreq_,
                  (throttle * .0005 + .0015) * pwmfreq_ * PI_HW_PWM_RANGE);
  gpioHardwarePWM(servopin_, pwmfreq_,
                  (steering * .0005 + .0015) * pwmfreq_ * PI_HW_PWM_RANGE);
}

bool PiGPIOCar::GetWheelMotion(float *, float *) {
  return false;
}

void PiGPIOCar::RunMainLoop(ControlCallback *cb) {
  timeval t0;
  gettimeofday(&t0, NULL);
  unsigned pwmusec = 1000000 / pwmfreq_;
  for (;;) {
    timeval t;
    gettimeofday(&t, NULL);
    // sleep until we are at t0+dt
    unsigned long sleepus = t0.tv_usec + 1000000 * t0.tv_sec + pwmusec -
                            t.tv_usec - 1000000 * t.tv_sec;
    usleep(sleepus);
    gettimeofday(&t, NULL);
    float dt = (t.tv_usec - t0.tv_usec)*1e-6 + t.tv_sec - t0.tv_sec;
    cb->OnControlFrame(this, dt);
  }
}