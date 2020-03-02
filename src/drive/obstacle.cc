#include "drive/obstacle.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

ObstacleDetector::ObstacleDetector() {
  ymask_rle_ = NULL;
  yanglemap_ = NULL;
  uvmask_rle_ = NULL;
  uvanglemap_ = NULL;
}

ObstacleDetector::~ObstacleDetector() {
  delete[] ymask_rle_;
  delete[] yanglemap_;
  delete[] uvmask_rle_;
  delete[] uvanglemap_;
}

bool ObstacleDetector::Open(const char *lut_fname) {
  uint8_t header[28];
  FILE *fp = fopen(lut_fname, "rb");
  if (!fp) {
    perror(lut_fname);
    return false;
  }

  if (fread(header, 1, sizeof(header), fp) != sizeof(header)) {
    fprintf(stderr, "ObstacleDetector::Open: short read\n");
    goto err;
  }

  if (header[0] != 'f' || header[1] != 'm' || header[2] != 'L' ||
      header[3] != 'U') {
    fprintf(stderr, "ObstacleDetector::Open: bad magic on floor map\n");
    goto err;
  }

  uint16_t h, w;
  uint32_t yanglesiz, yrlesiz, uvanglesiz, uvrlesiz;
  memcpy(&h, header + 8, 2);
  memcpy(&w, header + 10, 2);
  memcpy(&yanglesiz, header + 12, 4);
  memcpy(&yrlesiz, header + 16, 4);
  memcpy(&uvanglesiz, header + 20, 4);
  memcpy(&uvrlesiz, header + 24, 4);
  fprintf(stderr,
          "ObstacleDetector::Open: %dx%d imgsiz, %d Y angles, %d Y rle "
          "entries, %d UV angles, %d UV rle entries\n",
          w, h, yanglesiz, yrlesiz, uvanglesiz, uvrlesiz);

  ymask_rle_ = new uint16_t[yrlesiz];
  ymask_rlelen_ = yrlesiz;
  yanglemap_ = new int8_t[yanglesiz];
  uvmask_rle_ = new uint16_t[uvrlesiz];
  uvmask_rlelen_ = uvrlesiz;
  uvanglemap_ = new int8_t[uvanglesiz];
  if (fread(ymask_rle_, 2, yrlesiz, fp) != yrlesiz) {
    fprintf(stderr, "ObstacleDetector::Open: short read on y rle table\n");
    goto err;
  }

  if (fread(yanglemap_, 1, yanglesiz, fp) != yanglesiz) {
    fprintf(stderr, "ObstacleDetector::Open: short read on y angle table\n");
    goto err;
  }

  if (fread(uvmask_rle_, 2, uvrlesiz, fp) != uvrlesiz) {
    fprintf(stderr, "ObstacleDetector::Open: short read on uv rle table\n");
    goto err;
  }

  if (fread(uvanglemap_, 1, uvanglesiz, fp) != uvanglesiz) {
    fprintf(stderr, "ObstacleDetector::Open: short read on uv angle table\n");
    goto err;
  }

  fclose(fp);
  return true;

err:
  fclose(fp);
  return false;
}

void ObstacleDetector::Update(uint8_t *yuv420, uint8_t carthresh, uint8_t conethresh) {
  memset(black_sum_, 0, sizeof(black_sum_));
  memset(orange_sum_, 0, sizeof(orange_sum_));

  int rleptr = 0;
  int amptr = 0;
  uint8_t *y = yuv420;
  while (rleptr < ymask_rlelen_) {
    // read zero-len
    y += ymask_rle_[rleptr++];
    int n = ymask_rle_[rleptr++];
    while (n--) {
      if (*y < carthresh) {
        int a = yanglemap_[amptr];
        black_sum_[128+a] += carthresh - (*y);
        *y = 255;
      }
      y++;
      amptr++;
    }
  }

  uint8_t *v = yuv420 + 640*480 + 320*240;
  rleptr = 0;
  amptr = 0;
  while (rleptr < uvmask_rlelen_) {
    // read zero-len
    v += uvmask_rle_[rleptr++];
    int n = uvmask_rle_[rleptr++];
    while (n--) {
      if (*v > conethresh) {
        int a = uvanglemap_[amptr];
        orange_sum_[128+a] += (*v) - conethresh;
        *v = 255;
      }
      v++;
      amptr++;
    }
  }
}
