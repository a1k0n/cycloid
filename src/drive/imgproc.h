#ifndef DRIVE_IMGPROC_H_
#define DRIVE_IMGPROC_H_

#include <Eigen/Dense>
#include "drive/config.h"

// maps saved, output is 113 x 58
// uxrange (-58, 55) uyrange (2, 60) x0 -58 y0 2
namespace imgproc {
  // size determined by tools/mapgen
  static const int uxsiz = 113, uysiz = 58;
  static const float pixel_scale_m = 0.025;
  static const int ux0 = -58, uy0 = 2;

  // Returns a statically allocated object; not thread-safe
  int32_t *Reproject(const uint8_t *yuv);

  // TophatFilter destroys accumbuf
  bool TophatFilter(const DriverConfig &config, int32_t *accumbuf,
      Eigen::Vector3f *Bout, float *y_cout, Eigen::Matrix4f *Rkout,
      uint8_t *annotatedyuv);
}  // namespace imgproc

#endif  // DRIVE_IMGPROC_H_
