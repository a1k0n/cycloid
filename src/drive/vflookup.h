#ifndef DRIVE_VFLOOKUP_H_
#define DRIVE_VFLOOKUP_H_

#include <math.h>
#include <stdint.h>
#include <cmath>
#include <algorithm>

class ValueFuncLookup {
 public:
  ValueFuncLookup() {
    h_ = w_ = a_ = v_ = 0;
    scale_ = 1.;
    data_ = NULL;
  }
  ~ValueFuncLookup();

  bool Init();

  static float h2f(uint16_t h) {
    typedef union {
      uint32_t u;
      float f;
    } FP32;
    static const FP32 magic = {(254 - 15) << 23};
    FP32 o;

    o.u = (h & 0x7fff) << 13;   // exponent/mantissa bits
    o.f *= magic.f;             // exponent adjust
    o.u |= (h & 0x8000) << 16;  // sign bit
    return o.f;
  }

  float V(float x, float y, float theta, float v) {
    float ftheta = theta * a_ * 1.0/(2*M_PI);
    if (ftheta >= a_)
      ftheta -= a_;
    if (ftheta < 0)
      ftheta += a_;
    int itheta = std::floor(ftheta);
    ftheta -= itheta;
    // due to fp precision issues, we might still be rounded to a_ here
    if (itheta >= a_)
      itheta -= a_;
    float fv = std::min(std::max(v - vmin_, 0.0f), v_ - 1.0f);
    int iv = std::floor(fv);
    fv -= iv;
    float fx = x * scale_;
    int ix = std::floor(fx);
    fx -= ix;
    float fy = -y * scale_;
    int iy = std::floor(fy);
    fy -= iy;
    if (ix < 0 || ix >= w_ - 1 || iy < 0 || iy >= h_ - 1)
      return 1000.0f;

    int idx_xy = ix + iy * w_;
    int di = idx_xy + itheta * w_ * h_ + iv * w_ * h_ * a_;

    int nexttheta =
        itheta < a_ - 1 ? w_ * h_ : -w_ * h_ * (a_ - 1);

    //     vtyx
    float V0000 = h2f(data_[di]);
    float V0001 = h2f(data_[di + 1]);
    float V0010 = h2f(data_[di + w_]);
    float V0011 = h2f(data_[di + w_ + 1]);
    float V0100 = h2f(data_[di + nexttheta]);
    float V0101 = h2f(data_[di + nexttheta + 1]);
    float V0110 = h2f(data_[di + nexttheta + w_]);
    float V0111 = h2f(data_[di + nexttheta + w_ + 1]);
    float V1000 = V0000;
    float V1001 = V0001;
    float V1010 = V0010;
    float V1011 = V0011;
    float V1100 = V0100;
    float V1101 = V0101;
    float V1110 = V0110;
    float V1111 = V0111;
    if (iv < v_ - 1) {
      V1000 = h2f(data_[di + w_ * h_ * a_]);
      V1001 = h2f(data_[di + w_ * h_ * a_ + 1]);
      V1010 = h2f(data_[di + w_ * h_ * a_ + w_]);
      V1011 = h2f(data_[di + w_ * h_ * a_ + w_ + 1]);
      V1100 = h2f(data_[di + w_ * h_ * a_ + nexttheta]);
      V1101 = h2f(data_[di + w_ * h_ * a_ + nexttheta + 1]);
      V1110 = h2f(data_[di + w_ * h_ * a_ + nexttheta + w_]);
      V1111 = h2f(data_[di + w_ * h_ * a_ + nexttheta + w_ + 1]);
    }
    // lerp
    return (1 - fv) *
               ((1 - ftheta) * ((1 - fy) * ((1 - fx) * V0000 + fx * V0001) +
                                fy * ((1 - fx) * V0010 + fx * V0011)) +
                ftheta * ((1 - fy) * ((1 - fx) * V0100 + fx * V0101) +
                          fy * ((1 - fx) * V0110 + fx * V0111))) +
           fv * ((1 - ftheta) * ((1 - fy) * ((1 - fx) * V1000 + fx * V1001) +
                                 fy * ((1 - fx) * V1010 + fx * V1011)) +
                 ftheta * ((1 - fy) * ((1 - fx) * V1100 + fx * V1101) +
                           fy * ((1 - fx) * V1110 + fx * V1111)));
  }

 private:
  // height, width, number of angles, number of velocities
  int h_, w_, a_, v_;
  float scale_;  // meters / pixel
  float vmin_;
  uint16_t *data_;
};

#endif  // DRIVE_VFLOOKUP_H_
