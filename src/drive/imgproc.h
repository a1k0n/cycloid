#ifndef DRIVE_IMGPROC_H_
#define DRIVE_IMGPROC_H_

#include <Eigen/Dense>

namespace imgproc {
  // size determined by tools/mapgen
  static const int uxsiz = 112, uysiz = 56;

  // Returns a statically allocated object; not thread-safe
  int32_t *Reproject(const uint8_t *yuv);

  // TophatFilter destroys accumbuf
  bool TophatFilter(int32_t *accumbuf, Eigen::Vector3f *Bout,
      float *y_cout, Eigen::Matrix4f *Rkout);
}  // namespace imgproc

#endif  // DRIVE_IMGPROC_H_
