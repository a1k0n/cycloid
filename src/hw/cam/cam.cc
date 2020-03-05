#include "hw/cam/cam.h"

#include <stdlib.h>
#include <stdio.h>

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_component_wrapper.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

MMAL_POOL_T *Camera::camera_pool_ = NULL;
MMAL_POOL_T *Camera::encoder_pool_in_ = NULL;
MMAL_POOL_T *Camera::encoder_pool_out_ = NULL;
MMAL_COMPONENT_T *Camera::camera_ = NULL;
MMAL_COMPONENT_T *Camera::encoder_ = NULL;
CameraReceiver *Camera::receiver_ = NULL;

CameraReceiver::~CameraReceiver() {}

static void inport_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  // input port callback: just release the buffer
  mmal_buffer_header_release(buffer);
}

void Camera::ControlCallback(
    MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  fprintf(stderr, "Camera control callback cmd=0x%08x", buffer->cmd);

  mmal_buffer_header_release(buffer);
}

void Camera::BufferCallback(MMAL_PORT_T *port,
                            MMAL_BUFFER_HEADER_T *buffer) {
  if (buffer->length) {
    if (receiver_ != NULL) {
      mmal_buffer_header_mem_lock(buffer);
      receiver_->OnCameraFrame(buffer->data, buffer->length);
      mmal_buffer_header_mem_unlock(buffer);
    }
  }

  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled) {
    MMAL_STATUS_T status;
    MMAL_BUFFER_HEADER_T *new_buffer = mmal_queue_get(camera_pool_->queue);

    // and back to the port from there.
    if (new_buffer) {
      status = mmal_port_send_buffer(port, new_buffer);
    }

    if (!new_buffer || status != MMAL_SUCCESS)
      fprintf(stderr, "Unable to return the buffer to the camera port\n");
  }
}

void Camera::EncoderCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  if (buffer->length) {
    if (receiver_ != NULL) {
      mmal_buffer_header_mem_lock(buffer);
      receiver_->OnH264Frame(buffer->data, buffer->length);
      mmal_buffer_header_mem_unlock(buffer);
    }
  }

  mmal_buffer_header_release(buffer);
}

void Camera::EncodeFrame(uint8_t *buf, size_t len, bool iframe) {
  if (iframe) {
    mmal_port_parameter_set_boolean(encoder_->output[0],
                                    MMAL_PARAMETER_VIDEO_REQUEST_I_FRAME, 1);
  }

  MMAL_STATUS_T status;
  MMAL_BUFFER_HEADER_T *new_buffer = mmal_queue_get(encoder_pool_in_->queue);

  // and back to the port from there.
  if (new_buffer) {
    // TODO(a1k0n): is it possible to just send the video buffer itself,
    // without doing memcpy?
    memcpy(new_buffer->data, buf, len);
    new_buffer->length = len;
    status = mmal_port_send_buffer(encoder_->input[0], new_buffer);
    if (status != MMAL_SUCCESS) {
      fprintf(stderr, "mmal_port_send_buffer -> encoder: %d\n", status);
    }
  }

  if (!new_buffer || status != MMAL_SUCCESS)
    fprintf(stderr, "Unable to send buffer to encoder port\n");

  // refill empty output buffers
  {
    MMAL_BUFFER_HEADER_T *buffer;
    while ((buffer = mmal_queue_get(encoder_pool_out_->queue)) != NULL) {
      mmal_port_send_buffer(encoder_->output[0], buffer);
    }
  }
}

bool Camera::Init(int width, int height, int fps) {
  if (width & 31) {
    fprintf(stderr, "camera: width must be multiple of 32");
    return false;
  }

  if (height & 15) {
    fprintf(stderr, "camera: height must be multiple of 16");
    return false;
  }

  MMAL_STATUS_T status;

  if ((status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera_))
      != MMAL_SUCCESS) {
    fprintf(stderr, "cannot create mmal camera component\n");
    return false;
  }

  // Enable the camera, and tell it its control callback function
  status = mmal_port_enable(camera_->control, ControlCallback);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot enable camera\n");
    return false;
  }

  MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
    { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) }};
  cam_config.max_stills_w = width;
  cam_config.max_stills_h = height;
  cam_config.stills_yuv422 = 0;
  cam_config.one_shot_stills = 0;
  cam_config.max_preview_video_w = width;
  cam_config.max_preview_video_h = height;
  cam_config.num_preview_video_frames = 3;
  cam_config.stills_capture_circular_buffer_height = 0;
  cam_config.fast_preview_resume = 0;
  cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC;

  mmal_port_parameter_set(camera_->control, &cam_config.hdr);

  // use matrix metering mode
  MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = {
    {MMAL_PARAMETER_EXP_METERING_MODE, sizeof(meter_mode)},
    MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX
  };
  mmal_port_parameter_set(camera_->control, &meter_mode.hdr);


  MMAL_PORT_T *video_port = camera_->output[1];

  video_port->format->encoding = MMAL_ENCODING_I420;
  video_port->format->encoding_variant = MMAL_ENCODING_I420;
  video_port->format->es->video.width = width;
  video_port->format->es->video.height = height;
  video_port->format->es->video.crop.x = 0;
  video_port->format->es->video.crop.y = 0;
  video_port->format->es->video.crop.width = width;
  video_port->format->es->video.crop.height = height;
  video_port->format->es->video.frame_rate.num = fps;
  video_port->format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(video_port);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot set video port format\n");
    return false;
  }

  if (mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder_) !=
      MMAL_SUCCESS) {
    fprintf(stderr, "Cannot create video encoder\n");
    return false;
  }

  MMAL_PORT_T *encoder_input = encoder_->input[0];
  encoder_input->format->encoding = MMAL_ENCODING_I420;
  encoder_input->format->encoding_variant = MMAL_ENCODING_I420;
  encoder_input->format->es->video.width = width;
  encoder_input->format->es->video.height = height;
  encoder_input->format->es->video.crop.x = 0;
  encoder_input->format->es->video.crop.y = 0;
  encoder_input->format->es->video.crop.width = width;
  encoder_input->format->es->video.crop.height = height;
  // encoder_input->format->es->video.frame_rate.num = fps;
  // encoder_input->format->es->video.frame_rate.den = 1;
  if (mmal_port_format_commit(encoder_input) != MMAL_SUCCESS) {
    fprintf(stderr, "cannot commit encoder input format\n");
    return false;
  }

  MMAL_PORT_T *encoder_output = encoder_->output[0];
  mmal_format_copy(encoder_output->format, encoder_input->format);
  encoder_output->format->type = MMAL_ES_TYPE_VIDEO;
  encoder_output->format->encoding = MMAL_ENCODING_H264;
  encoder_output->format->bitrate = 8000000;

  encoder_input->buffer_size = encoder_input->buffer_size_recommended;
  encoder_input->buffer_num = encoder_input->buffer_num_min;
  encoder_output->buffer_size = encoder_output->buffer_size_recommended;
  encoder_output->buffer_num = encoder_output->buffer_num_min;
  if (mmal_port_format_commit(encoder_output) != MMAL_SUCCESS) {
    fprintf(stderr, "cannot commit encoder output format\n");
    return false;
  }

  if (mmal_port_enable(encoder_input, inport_cb) != MMAL_SUCCESS) {
    fprintf(stderr, "Failed to enable encoder input\n");
    return false;
  }

  if (mmal_port_enable(encoder_output, EncoderCallback) != MMAL_SUCCESS) {
    fprintf(stderr, "Failed to setup encoder output callback\n");
    return false;
  }

  if (mmal_component_enable(encoder_) != MMAL_SUCCESS) {
    fprintf(stderr, "failed to enable encoder component\n");
    return false;
  }

  // One video buffer only; we definitely do not want to queue up old frames if
  // we're running behind
  video_port->buffer_num = 1;

  status = mmal_component_enable(camera_);
  if (status != MMAL_SUCCESS) {
    fprintf(stderr, "cannot enable camera\n");
    return false;
  }

  // the red LED should be on now.
  fprintf(stderr, "video port %s: %d buffers x %d bytes\n",
          video_port->name, video_port->buffer_num, video_port->buffer_size);

  // Create pool of buffer headers for the output port to consume
  camera_pool_ = mmal_port_pool_create(
      video_port, video_port->buffer_num, video_port->buffer_size);
  if (!camera_pool_) {
    fprintf(stderr, "cannot create buffer header pool for video port\n");
    return false;
  }

  encoder_pool_in_ = mmal_port_pool_create(
      encoder_input, encoder_input->buffer_num, encoder_input->buffer_size);
  if (!encoder_pool_in_) {
    fprintf(stderr, "cannot create buffer header pool for encoder in port\n");
    return false;
  }

  encoder_pool_out_ = mmal_port_pool_create(
      encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
  if (!encoder_pool_out_) {
    fprintf(stderr, "cannot create buffer header pool for encoder out port\n");
    return false;
  }

  if (mmal_port_enable(video_port, BufferCallback) != MMAL_SUCCESS) {
    fprintf(stderr, "Failed to setup camera output\n");
    return false;
  }

  // set shutter speed to auto (it probably was already?)
  if (mmal_port_parameter_set_uint32(
          camera_->control, MMAL_PARAMETER_SHUTTER_SPEED,
          0) != MMAL_SUCCESS) {
    fprintf(stderr, "cannot set shutter speed\n");
    return false;
  }

  // Send one buffer to the camera port
  // (should be only one outstanding buffer at a time)
  {
    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(camera_pool_->queue);
    if (!buffer) {
      fprintf(stderr, "Unable to get a required buffer from pool queue\n");
      return false;
    }

    if (mmal_port_send_buffer(video_port, buffer) != MMAL_SUCCESS) {
      fprintf(stderr, "Unable to send a buffer to camera output port\n");
      return false;
    }
  }

  return true;
}

bool Camera::StartRecord(CameraReceiver *receiver) {
  receiver_ = receiver;
  MMAL_PORT_T *video_port = camera_->output[1];
  // enable capturing
  if (mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_CAPTURE, 1)
      != MMAL_SUCCESS) {
    fprintf(stderr, "failed to start capture\n");
    return false;
  }


  return true;
}

bool Camera::StopRecord() {
  MMAL_PORT_T *video_port = camera_->output[1];
  // enable capturing
  if (mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_CAPTURE, 0)
      != MMAL_SUCCESS) {
    fprintf(stderr, "failed to stop capture\n");
    return false;
  }
  receiver_ = NULL;

  return true;
}
