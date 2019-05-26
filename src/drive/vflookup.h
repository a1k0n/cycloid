#ifndef DRIVE_VFLOOKUP_H_
#define DRIVE_VFLOOKUP_H_

#include <math.h>
#include <cmath>

class ValueFuncLookup {
 public:
  ValueFuncLookup() {
    h_ = w_ = a_ = 0;
    scale_ = 1.;
    data_ = NULL;
  }
  ~ValueFuncLookup();

  bool Init();
  float V(float x, float y, float theta) {
    float a = theta*a_/(2*M_PI);
    x *= scale_;
    y *= -scale_;
    if (x < 0 || x > w_ - 2 || y < 0 || y > h_ - 2) return 1000.;

    int x0 = std::floor(x);
    int y0 = std::floor(y);
    int a0 = std::floor(a);
    float dx = x - x0;
    float dy = y - y0;
    float da = a - a0;
    a0 %= a_;
    if (a0 < 0) a0 += a_;
    int a1 = a0 + 1;
    if (a1 == a_) a1 -= a_;

    // trilinear interpolation
    int ia0 = a0*w_*h_ + y0*w_ + x0;
    int ia1 = a1*w_*h_ + y0*w_ + x0;
    float v000 = data_[ia0];
    float v001 = data_[ia0 + 1];
    float v00 = (1-dx)*v000 + dx*v001;
    float v010 = data_[ia0 + w_];
    float v011 = data_[ia0 + w_ + 1];
    float v01 = (1-dx)*v010 + dx*v011;
    float v0 = (1-dy)*v00 + dy*v01;
    float v100 = data_[ia1];
    float v101 = data_[ia1 + 1];
    float v10 = (1-dx)*v100 + dx*v101;
    float v110 = data_[ia1 + w_];
    float v111 = data_[ia1 + w_ + 1];
    float v11 = (1-dx)*v110 + dx*v111;
    float v1 = (1-dy)*v10 + dy*v11;
    return (1-da)*v0 + da*v1;
  }

 private:
  // height, width, number of angles
  int h_, w_, a_;
  float scale_;  // meters / pixel
  __fp16 *data_;
};

#endif  // DRIVE_VFLOOKUP_H_