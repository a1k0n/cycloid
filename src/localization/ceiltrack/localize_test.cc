#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <zlib.h>

#include "localization/ceiltrack/ceiltrack.h"

const float CEIL_HEIGHT = 8.25;
const float X_GRID = 10/CEIL_HEIGHT;
const float Y_GRID = 12/CEIL_HEIGHT;

int main() {
  CeilingTracker ctrack;

  if (!ctrack.Open(TESTDATA_PATH "/lut.bin")) {
    fprintf(stderr, "ceiltrack localizer init fail\n");
    return 1;
  }

  uint8_t y[480*640];
  gzFile zf = gzopen(TESTDATA_PATH "/data.raw.gz", "rb");
  if (zf == NULL) {
    perror("testdata");
    return 1;
  }

  FILE *gf = fopen(TESTDATA_PATH "/golden.txt", "r");
  if (gf == NULL) {
    perror("golden.txt");
    return 1;
  }

  int frame = 0;
  float B[3];
  const float eps = 1e-3;
  double trackusec = 0;
  int trackiters = 0;
  for (int iter = 0; iter < 10; iter++) {
    memset(B, 0, sizeof(B));
    while (gzread(zf, y, sizeof(y)) == sizeof(y)) {
      timeval tv0, tv1;
      gettimeofday(&tv0, NULL);
      ctrack.Update(y, 240, X_GRID, Y_GRID, B, 6, frame == 0 && iter == 0);
      gettimeofday(&tv1, NULL);
      trackusec +=
          (tv1.tv_sec - tv0.tv_sec) * 1e6 + (tv1.tv_usec - tv0.tv_usec);
      trackiters += 1;  // 6;
      float gx, gy, gt;
      if (fscanf(gf, "%f %f %f\n", &gx, &gy, &gt) != 3) {
        fprintf(stderr, "golden.txt parse error");
        return 1;
      }
      if (fabs(gx - B[0]) > eps || fabs(gy - B[1]) > eps ||
          fabs(gt - B[2]) > eps) {
        fprintf(stderr, "test error frame %d (%f %f %f) should be (%f %f %f)\n",
                frame, B[0], B[1], B[2], gx, gy, gt);
        return 1;
      }
      frame++;
    }
    gzrewind(zf);
    rewind(gf);
  }

  printf("validated %d frames\n", frame);
  printf("%f usec/frame\n", trackusec / trackiters);
  return 0;
}
