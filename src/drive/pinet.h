#ifndef DRIVE_PINET_H_
#define DRIVE_PINET_H_

#include <string.h>

const int HIDDEN = 16;
// FIXME
const int NEMBED = 315;

class PiNetwork {
 public:
  PiNetwork() {
    memset(this, 0, sizeof(*this));
  }

  bool Load(const char *fname);

  void Action(int ix, float v, float w, float xl, float yl, float cl, float sl,
              float tscale, float sscale, float *u_throttle, float *u_steering);

 private:
  float emb[NEMBED][HIDDEN];
  float state_weight1[HIDDEN][6];
  float state_bias1[HIDDEN];
  float linear_weight1[HIDDEN][HIDDEN];
  float linear_bias1[HIDDEN];
  float mu_weight[2][HIDDEN];
  float mu_bias[2];
  float logstd_weight[2][HIDDEN];
  float logstd_bias[2];
};

#endif