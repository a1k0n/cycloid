#include <stdio.h>
#include <string.h>
#include "coneslam/imgproc.h"

#include "drive/config.h"

static const char *RECFILE =
  "../design/coneslam/home20180804/cycloid-20180804-194750.rec";
int main() {
  FILE *fp = fopen(RECFILE, "rb");
  if (!fp) {
    perror(RECFILE);
    return 1;
  }

  DriverConfig config;
  config.Load();

  int frame = 0;
  int xbuf[10];
  uint8_t framebuf[55 + 640*(480+240)];
  float thetabuf[10];
  while (fread(framebuf, sizeof(framebuf), 1, fp) > 0) {
    float gyroz;
    memcpy(&gyroz, framebuf+26+8, 4);
    printf("%d: gyroz=%f ", frame, gyroz);
    int ncones = coneslam::FindCones(/*yuvimg=*/framebuf+55, /*thresh=*/config.cone_thresh, /*gyroz=*/gyroz, /*nout=*/10, /*x_out*/xbuf, /*bearing_out=*/thetabuf);
    if (ncones) {
      for (int i = 0; i < ncones; i++) {
        printf("[%d]%f ", xbuf[i], thetabuf[i]);
      }
    }
    frame++;
    printf("\n");
  }
  fclose(fp);

  return 0;
}
