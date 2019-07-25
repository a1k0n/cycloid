#ifndef CEILTRACK_CEILTRACK_H_
#define CEILTRACK_CEILTRACK_H_

#include <stdint.h>

class CeilingTracker {
 public:
  CeilingTracker() {}

  bool Open(const char *lut_fname);

  // Update x, y, theta estimate from greyscale image, returning cost
  // any pixels >thresh are assumed to be ceiling light pixels
  float Update(const uint8_t *img, uint8_t thresh, float xgrid, float ygrid,
               float *xytheta, int niter, bool verbose);

 private:
  uint16_t *mask_rle_;
  int mask_rlelen_;
  __fp16 *uvmap_;
  int uvmaplen_;
};

#endif // CEILTRACK_CEILTRACK_H_
