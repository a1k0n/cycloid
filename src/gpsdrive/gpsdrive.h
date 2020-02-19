#ifndef GPSDRIVE_GPSDRIVE_H_
#define GPSDRIVE_GPSDRIVE_H_

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

#include "gpsdrive/config.h"
#include "hw/car/car.h"
#include "hw/gps/ubx.h"
#include "hw/input/input.h"

class FlushThread;
class IMU;
class INIReader;
class JoystickInput;
class UIDisplay;

class GPSDrive : public ControlListener,
                 public JoystickListener,
                 public NavListener {
 public:
  GPSDrive(FlushThread *ft, IMU *imu, JoystickInput *js, UIDisplay *disp);
  ~GPSDrive();

  bool Init(const INIReader &ini);

  void Quit();

  // ControlListener
  virtual bool OnControlFrame(CarHW *car, float dt);

  // NavListener
  virtual void OnNav(const nav_pvt &nav);

  // JoystickListener
  virtual void OnDPadPress(char direction);
  virtual void OnButtonPress(char button);
  virtual void OnButtonRelease(char button);
  virtual void OnAxisMove(int axis, int16_t value);

 private:
  void StartRecording();
  void StopRecording();

  DriverConfig config_;
  FlushThread *flush_thread_;
  IMU *imu_;
  JoystickInput *js_;
  UIDisplay *display_;
  int ubx_fd_;
  bool done_;

  int16_t js_throttle_, js_steering_;

  static void *gpsThread(void *);
  pthread_t gps_thread_;

  FILE *record_fp_;
  pthread_mutex_t record_mut_;
};

#endif  // GPSDRIVE_GPSDRIVE_H_
