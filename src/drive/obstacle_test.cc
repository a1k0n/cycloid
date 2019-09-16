#include "drive/obstacle.h"
#include <stdio.h>
#include <zlib.h>

int main() {
  ObstacleDetector d;
  if (!d.Open("testdata/floorlut.bin")) {
    return 1;
  }

  gzFile fp = gzopen("testdata/obsframe.yuv420.gz", "rb");
  if (!fp) {
    perror("testdata/");
  }
  uint8_t yuv420[640*480 + 320*240*2];
  if (gzread(fp, yuv420, sizeof(yuv420)) != sizeof(yuv420)) {
    fprintf(stderr, "short read on testdata frame");
    return 1;
  }
  gzclose(fp);

  d.Update(yuv420, 40, 150);
  const int32_t *p0 = d.GetCarPenalties();
  const int32_t *p1 = d.GetConePenalties();
  for (int i = 0; i < 256; i++) {
    if (p0[i] != 0 || p1[i] != 0) {
      printf("%4d: %5d %5d\n", i-128, p0[i], p1[i]);
    }
  }

  return 0;
}