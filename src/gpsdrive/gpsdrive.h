#ifndef GPSDRIVE_GPSDRIVE_H_
#define GPSDRIVE_GPSDRIVE_H_

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

#include <Eigen/Dense>

#include "gpsdrive/config.h"
#include "gpsdrive/trajtrack.h"
#include "hw/car/car.h"
#include "hw/gps/ubx.h"
#include "hw/input/input.h"

class FlushThread;
class Magnetometer;
class IMU;
class INIReader;
class JoystickInput;
class UIDisplay;

class GPSDrive : public ControlListener,
                 public JoystickListener,
                 public NavListener {
 public:
  GPSDrive(FlushThread *ft, IMU *imu, Magnetometer *mag, JoystickInput *js,
           UIDisplay *disp);
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
  void UpdateDisplay();

  DriverConfig config_;
  TrajectoryTracker raceline_;
  FlushThread *flush_thread_;
  IMU *imu_;
  Magnetometer *mag_;
  JoystickInput *js_;
  UIDisplay *display_;
  int ubx_fd_;
  bool done_;

  int16_t js_throttle_, js_steering_;

  bool autodrive_;
  float ierr_k_;
  float ierr_v_;
  float last_v_, last_w_;
  float last_u_esc_;
  int brake_count_;

  Eigen::Vector3f gyro_last_, gyro_bias_;

  int32_t lat_, lon_;
  int32_t ref_lat_, ref_lon_;
  Eigen::Vector3f gps_v_;
  int numSV_;
  float mscale_lat_, mscale_lon_;
  float ye_, psie_, k_;
  float autodrive_k_, autodrive_v_;

  int config_item_;
  bool x_down_, y_down_;

  static void *gpsThread(void *);
  pthread_t gps_thread_;

  FILE *record_fp_;
  pthread_mutex_t record_mut_;
};

#endif  // GPSDRIVE_GPSDRIVE_H_
