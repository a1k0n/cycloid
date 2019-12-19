#include "lens/fisheye.h"

#include <math.h>

// 2x Newton steps to invert distortion
static inline float solvetheta(float thetad, float k1) {
  float theta = thetad;
  theta += (theta * (k1 * theta * theta + 1) - thetad) /
           (-3 * k1 * theta * theta - 1);
  theta += (theta * (k1 * theta * theta + 1) - thetad) /
           (-3 * k1 * theta * theta - 1);
  return theta;
}

float* FisheyeLens::GenUndistortedPts(int w, int h) const {
  float* buf = new float[w * h * 3];
  int idx = 0;
  for (int j = 0; j < h; j++) {
    float v = (j - cy) / fy;
    for (int i = 0; i < w; i++) {
      float u = (i - cx) / fx;
      float r = sqrtf(u * u + v * v);
      float a = u / r;
      float b = v / r;
      float theta = solvetheta(r, k1);
      float t = 1.0 / tanf(M_PI_2 - theta);
      float at = fabs(t);
      buf[idx++] = a * at;
      buf[idx++] = b * at;
      buf[idx++] = signbit(t) ? -1 : 1;
    }
  }
  return buf;
}

void FisheyeLens::DistortPoint(float x, float y, float z, float* u, float* v) const {
  float r = sqrtf(x * x + y * y);
  float theta = atan2f(r, z);
  float dist = theta * (1 + k1 * theta * theta) / r;
  *u = fx * dist * x + cx;
  *v = fy * dist * y + cy;
}