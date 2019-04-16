#include <math.h>
#include "coneslam/fisheye.h"

namespace fisheye {

// how much visualizer shows under the horizon vs above it (out of 56 pixels)
static const int vis_y0 = 10;

Fisheye::Fisheye() {
  float w, h, fx, fy, cx, cy, k1;
  float theta = M_PI / 2;
  theta = theta * (1 + k1 * theta * theta);  // apply distortion

  int npix =
      rint(M_PI * 2 * fx * theta);  // get circumference of horizon in pixels
  lut_linear_ = new uint32_t[npix];

  // also build a remap table just for visualization on a display
  // 320x56 front, 320x56 rear
  lut_remap_ = new uint32_t[320 * 112];
  for (int j = 0; j < 56; j++) {
    // get latitude angle from pixel location
    float lat = M_PI / 2 - (j - vis_y0) / fx;
    for (int i = 0; i < 320; i++) {
      
    }
  }
}

}  // namespace fisheye