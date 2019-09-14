
#ifndef DRIVE_DRIVER_H_
#define DRIVE_DRIVER_H_

#include "drive/config.h"
#include "drive/controller.h"
#include "hw/cam/cam.h"
#include "hw/car/car.h"
#include "hw/input/input.h"

class CeilingTracker;
class DriveController;
class DriverConfig;
class FlushThread;
class IMU;
class JoystickInput;
class UIDisplay;

class Driver : public CameraReceiver,
               public ControlCallback,
               public InputReceiver {
 public:
  // FIXME(a1k0n): CeilingTracker -> Localizer
  Driver(const INIReader &ini, CeilingTracker *ceil, FlushThread *ft, IMU *imu,
         JoystickInput *js, UIDisplay *disp);
  ~Driver();

  virtual void OnCameraFrame(uint8_t *buf, size_t length);
  virtual bool OnControlFrame(CarHW *car, float dt);

  virtual void OnDPadPress(char direction);

  virtual void OnButtonPress(char button);
  virtual void OnButtonRelease(char button);

  virtual void OnAxisMove(int axis, int16_t value);

  void Quit() { done_ = true; }

 private:
  bool StartRecording(const char *fname, int frameskip);
  bool IsRecording();
  void StopRecording();

  void UpdateFromCamera(uint8_t *buf, float dt);

  void UpdateDisplay();

  void QueueRecordingData(const timeval &t, uint8_t *buf, size_t length);

  CeilingTracker *ceiltrack_;
  DriveController controller_;
  DriverConfig config_;
  FlushThread *flush_thread_;
  IMU *imu_;
  JoystickInput *js_;
  UIDisplay *display_;

  bool autodrive_;
  bool done_;
  int frame_;

  const char *name;
  int output_fd_;
  int frameskip_;
  struct timeval last_t_;
  int16_t js_throttle_, js_steering_;

  Eigen::Vector3f gyro_last_, gyro_bias_;

  static const char *configmenu[];
  static const int N_CONFIGITEMS;
  int config_item_;
  bool x_down_, y_down_;
};

#endif  // DRIVE_DRIVER_H_
