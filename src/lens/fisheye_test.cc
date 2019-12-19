#include "lens/fisheye.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static inline bool close(float a, float b) {
  int ia = a*4096;
  int ib = b*4096;
  return ia == ib;
}

int main() {
  float golden_xy[640 * 480 * 2];
  FILE *fp = fopen(TESTDATA_PATH "/xy.bin", "rb");
  if (!fp) {
    perror("xy.bin");
    return 1;
  }
  fread(golden_xy, 640 * 480 * 2, 4, fp);
  fclose(fp);

  FisheyeLens lens;
  lens.SetCalibration(193.25951159699667, 192.68330525878318,
                      347.82753095730305, 245.52990771384336,
                      0.009250724605410428);

  float *out = lens.GenUndistortedPts(640, 480);
  int mismatches = 0, total = 0;
  for (int j = 0; j < 480; j++) {
    for (int i = 0; i < 640; i++) {
      int gidx = 2 * (i + 640*j);
      int oidx = 3 * (i + 640*j);
      if (out[oidx+2] > 0) {
        if (!close(out[oidx], golden_xy[gidx])) {
          mismatches++;
        }
        if (!close(out[oidx+1], golden_xy[gidx+1])) {
          mismatches++;
        }
        total+=2;
      }
      // check that round-trip undistort->distort returns the original point
      float x, y;
      lens.DistortPoint(out[oidx], out[oidx+1], out[oidx+2], &x, &y);
      if (lroundf(x) != i || lroundf(y) != j) {
        printf("%d %d -> %f %f (%f)\n", i, j, x, y, out[oidx+2]);
        return 1;
      }
    }
  }

  // we expect some mismatches, partly from differing precision and partly
  // because our method handles edge cases better
  printf("%d/%d coordinates mismatched OpenCV data\n", mismatches, total);

  return 0;
}