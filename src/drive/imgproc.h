#ifndef DRIVE_IMGPROC_H_
#define DRIVE_IMGPROC_H_

#include <Eigen/Dense>
#include "drive/config.h"

// maps saved, output is 111 x 56
// uxrange (-57, 54) uyrange (2, 58) x0 -57 y0 2


namespace imgproc {
  // size determined by tools/mapgen
  static const int uxsiz = 111, uysiz = 56;
  static const float pixel_scale_m = 0.025;
  static const int ux0 = -57, uy0 = 2;

  // Returns a statically allocated object; not thread-safe
  int32_t *Reproject(const uint8_t *yuv);

  // TophatFilter destroys accumbuf
  bool TophatFilter(const DriverConfig &config, int32_t *accumbuf,
      Eigen::Vector3f *Bout, float *y_cout, Eigen::Matrix4f *Rkout,
      uint8_t *annotatedyuv);
}  // namespace imgproc

#endif  // DRIVE_IMGPROC_H_
