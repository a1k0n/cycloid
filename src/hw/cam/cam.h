#ifndef HW_CAM_CAM_H_
#define HW_CAM_CAM_H_

#include <stdint.h>
#include <stdlib.h>

class CameraReceiver {
 public:
  virtual ~CameraReceiver();
  virtual void OnCameraFrame(uint8_t *buf, size_t len) = 0;
  virtual void OnH264Frame(uint8_t *buf, size_t len) = 0;
};

struct MMAL_BUFFER_HEADER_T;
struct MMAL_COMPONENT_T;
struct MMAL_POOL_T;
struct MMAL_PORT_T;

class Camera {
 public:
  static bool Init(int width, int height, int fps);

  static bool StartRecord(CameraReceiver *receiver);
  static bool StopRecord();

  static void EncodeFrame(uint8_t *buf, size_t len);

 private:
  static MMAL_COMPONENT_T *camera_;
  static MMAL_COMPONENT_T *encoder_;
  static MMAL_POOL_T *camera_pool_;
  static MMAL_POOL_T *encoder_pool_;
  static CameraReceiver *receiver_;

  static void ControlCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
  static void BufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
  static void EncoderCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
};

#endif  // HW_CAM_CAM_H_
