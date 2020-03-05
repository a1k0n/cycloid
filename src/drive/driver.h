
#ifndef DRIVE_DRIVER_H_
#define DRIVE_DRIVER_H_

#include "drive/config.h"
#include "drive/controller.h"
#include "drive/obstacle.h"
#include "hw/cam/cam.h"
#include "hw/car/car.h"
#include "hw/input/input.h"
#include "io/flushthread.h"
#include "lens/fisheye.h"
#include "localization/ceiltrack/ceiltrack.h"

class DriveController;
class DriverConfig;
class FlushThread;
class IMU;
class JoystickInput;
class UIDisplay;

class Driver : public CameraReceiver,
               public ControlListener,
               public JoystickListener {
 public:
  // FIXME(a1k0n): CeilingTracker -> Localizer
  Driver(IMU *imu, JoystickInput *js, UIDisplay *disp);
  ~Driver();

  bool Init(const INIReader &ini);

  virtual void OnCameraFrame(uint8_t *buf, size_t length);
  virtual void OnH264Frame(uint8_t *buf, size_t length);
  virtual bool OnControlFrame(CarHW *car, float dt);

  virtual void OnDPadPress(char direction);

  virtual void OnButtonPress(char button);
  virtual void OnButtonRelease(char button);

  virtual void OnAxisMove(int axis, int16_t value);

  void Quit() { done_ = true; }

 private:
  bool StartRecording(const char *logname, const char *h264name);
  bool IsRecording();
  void StopRecording();

  void UpdateFromCamera(uint8_t *buf, float dt);

  void UpdateDisplay();

  void QueueRecordingData(const timeval &t);

  FisheyeLens lens_;
  CeilingTracker ceiltrack_;
  ObstacleDetector obstacledetect_;
  DriveController controller_;
  DriverConfig config_;
  IMU *imu_;
  JoystickInput *js_;
  UIDisplay *display_;

  FlushThread flush_thread_;

  bool autodrive_;
  bool done_;
  int log_frame_;

  int log_fd_, h264_fd_;
  struct timeval last_t_, last_lap_;
  int16_t js_throttle_, js_steering_;

  Eigen::Vector3f gyro_last_, gyro_bias_;
  Eigen::Vector3f accel_last_, accel_bias_;

  int config_item_;
  bool x_down_, y_down_;
};

#endif  // DRIVE_DRIVER_H_
