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

struct ControlOutput {
  uint8_t leds;
  float u_esc, u_servo;

  ControlOutput() {
    leds = 0;
    u_esc = 0;
    u_servo = 0;
  }

  void Set(uint8_t leds, float u_esc, float u_servo) {
    this->leds = leds;
    this->u_esc = u_esc;
    this->u_servo = u_servo;
  }
};

struct StateObservation {
  float vx;  // forward velocity
  float w;  // yaw rate
  StateObservation() { vx = 0; w = 0; }
};

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
  void UpdateControls(float in_throttle, float in_steering, bool radio_safe,
                      const StateObservation &obs, float dt,
                      ControlOutput *out);

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

  // controller state
  bool autodrive_;
  float ierr_k_;
  float ierr_v_;
  float last_v_, last_w_;
  // TODO: circular buffer of observations, so we determine yaw rate
  // error from ~11 frames ago
  float last_target_k_;
  int brake_count_;

  // IMU state
  Eigen::Vector3f gyro_last_, gyro_bias_;

  // GPS state
  int32_t lat_, lon_;
  int32_t ref_lat_, ref_lon_;
  Eigen::Vector3f gps_v_;
  int numSV_;
  float mscale_lat_, mscale_lon_;

  // TODO: remove these and compute in control frame
  // replace with float x_, y_, theta_;
  float navheading_, navheading_last_;  // heading from nav (radians)
  float heading_;  // heading (radians) complementary filter state
  float heading_rollover_;  // adjustment for extra revolutions of heading (dumb hack)
  bool heading_init_;  // flag used to bootstrap initial heading

  float ye_, psie_, k_;
  float autodrive_k_, autodrive_v_;

  ControlOutput control_hist_[16];
  int control_hist_ptr_;
  StateObservation state_hist_[16];
  int state_hist_ptr_;

  // game controller / display state
  int config_item_;
  bool x_down_, y_down_;

  static void *gpsThread(void *);
  pthread_t gps_thread_;

  FILE *record_fp_;
  pthread_mutex_t record_mut_;
};

#endif  // GPSDRIVE_GPSDRIVE_H_
