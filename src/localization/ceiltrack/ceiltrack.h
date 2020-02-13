#ifndef LOCALIZATION_CEILTRACK_CEILTRACK_H_
#define LOCALIZATION_CEILTRACK_CEILTRACK_H_

#include <stdint.h>

#include <vector>

#include "lens/fisheye.h"

class CeilingTracker {
 public:
  CeilingTracker() {}
  CeilingTracker(const FisheyeLens &lens, float camtilt) {
    Init(lens, camtilt);
  }

  bool Init(const FisheyeLens &lens, float camtilt);

  // Update x, y, theta estimate from greyscale image, returning cost
  // any pixels >thresh are assumed to be ceiling light pixels
  float Update(const uint8_t *img, uint8_t thresh, float xgrid, float ygrid,
               float *xytheta, int niter, bool verbose);

  void GetMatchedGrid(const FisheyeLens &lens, const float *xytheta,
                      float xgrid, float ygrid,
                      std::vector<std::pair<float, float>> *out) const;

 private:
  uint16_t *mask_rle_;
  int mask_rlelen_;
  float *uvmap_;
  int uvmaplen_;

  float camtilt_;
};

#endif  // LOCALIZATION_CEILTRACK_CEILTRACK_H_
