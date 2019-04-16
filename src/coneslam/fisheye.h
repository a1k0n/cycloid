#ifndef CONESLAM_FISHEYE_H_
#define CONESLAM_FISHEYE_H_

#include <stdint.h>

namespace coneslam {

// Camera calibration for fisheye lenses;
// generates lookup tables and reprojects images

class Fisheye {
 public:
  // construct fisheye lens for image of given size (w, h),
  // focal length (fx, fy),
  // optical center (cx, cy),
  // and distortion k1
  Fisheye();

  ~Fisheye() {
    delete[] lut_linear_;
    delete[] lut_remap_;
  }

 private:
  uint32_t *lut_linear_;
  int lut_xres_;  // horizontal resolution of linear LUT

  uint32_t *lut_remap_;
  int remap_w_, remap_h_;  // remap size
};

}  // namespace coneslam

#endif  // CONESLAM_FISHEYE_H_
