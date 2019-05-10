#ifndef DRIVE_VFLOOKUP_H_
#define DRIVE_VFLOOKUP_H_

#include <math.h>

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
    int a = std::floor(theta*a_/(2*M_PI));
    a %= a_;
    if (a < 0) a += a_;
    x *= scale_;
    y *= -scale_;
    if (x < 0 || x > w_ - 1 || y < 0 || y > h_ - 1) return 1000.;

    // TODO(asloane): interpolation
    int idx = a*w_*h_ + std::floor(y)*w_ + std::floor(x);
    return data_[idx];
  }

 private:
  // height, width, number of angles
  int h_, w_, a_;
  float scale_;  // meters / pixel
  __fp16 *data_;
};

#endif  // DRIVE_VFLOOKUP_H_
