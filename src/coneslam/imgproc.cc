#include <algorithm>
#include <stdio.h>
#include "coneslam/imgproc.h"

namespace coneslam {

namespace lut {
#include "lut.h"
}  // namespace lut

using lut::conedetect_turn_slope;
using lut::conedetect_y_offset;
using lut::conedetect_vpy;
using lut::conedetect_width;
using lut::conedetect_LUT;

float GetVpy() {
  return lut::conedetect_vpy;
}

int FindCones(const uint8_t *yuvimg, int thresh, float gyroz, int nout,
    int *x_out, float *bearing_out) {
  int32_t accumbuf[321];
  if (gyroz > 1.9) gyroz = 1.9;
  if (gyroz < -1.9) gyroz = -1.9;
  float y0 = -conedetect_turn_slope * gyroz + conedetect_vpy*0.5;
  float yinc = 2*conedetect_turn_slope * gyroz / 320.0;
  const uint8_t *imgv = yuvimg + 640*600;
  // convolve row of pixels with -1, -1, -1, 2, 2, 2, -1, -1, -1

  // compute a running sum of the horizintal data in accumbuf
  uint32_t xsum = 0;
  float y = y0;
  accumbuf[0] = 0;
  for (int i = 0; i < 320; i++) {
    int yi = static_cast<int>(y0) * 320;
    for (int j = 0; j < conedetect_width*320; j+=320) {
      // filter out blue V channel values, we want orange only
      // (this prevents strong blue stripes from forming "anti-cones")
      xsum += std::max(imgv[yi+j+i], (uint8_t)128);
      accumbuf[i+1] = xsum;
    }
    y += yinc;
  }

  // detect activations
  bool A[320 - 10];
  for (int i = 0; i < 320-10; i++) {
    // convolve with -1, -1, -1, 2, 2, 2, -1, -1, -1
    int a = 3*(accumbuf[i + 6] - accumbuf[i + 3])
      - (accumbuf[i + 9] - accumbuf[i]);
    A[i] = a > thresh;
  }

  // now spread them out for non-maximal suppression
  for (int j = 0; j < 5; j++) {
    for (int i = 0; i < 320-11; i++) {
      A[i] |= A[i+1];
    }
    for (int i = 320-11; i > 0; i--) {
      A[i] |= A[i-1];
    }
  }

  // now find the runs
  int l = -1;
  int outputs = 0;
  for (int i = 0; i < 320-10; i++) {
    if (!A[i] && l != -1) {
      int center = i-1+l + 9;  // technically ((i-1) + l) / 2 is the center
      // but the center is in 0..640, not 0..320, and we have an offset of 4
      // from the convolution
      x_out[outputs] = center;
      int lut_y = y0+yinc*center*0.5 + conedetect_y_offset;
      if (lut_y >= (int)(sizeof(conedetect_LUT) / 640)) {
        fprintf(stderr, "panic: lut_y = %d, too big gyroz=%f y0=%f yinc=%f", lut_y, gyroz, y0, yinc);
        lut_y = (sizeof(conedetect_LUT) / 640) - 1;
      }
      if (lut_y < 0) {
        fprintf(stderr, "panic: lut_y = %d, < 0? gyroz=%f y0=%f yinc=%f", lut_y, gyroz, y0, yinc);
        lut_y = 0;
      }
      bearing_out[outputs] = conedetect_LUT[lut_y*640 + center];
      l = -1;
      outputs++;
      if (outputs == nout) {
        return outputs;
      }
    } else if (A[i] && l == -1) {
      l = i;
    }
  }
  return outputs;
}

}  // namespace coneslam
