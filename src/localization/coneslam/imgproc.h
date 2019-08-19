#ifndef CONESLAM_IMGPROC_H_
#define CONESLAM_IMGPROC_H_

#include <stdint.h>

namespace coneslam {

// scan horizon of input 640x480 YUV image for orange cones, write
// bearing angle to each cone in bearing_out[:nout], return number found
// also tilt the input according to gyroz
int FindCones(const uint8_t *yuvimg, int thresh, float gyroz, int nout,
    int *x_out, float *bearing_out);

float GetVpy();

}  // namespace coneslam

#endif  // CONESLAM_IMGPROC_H_
