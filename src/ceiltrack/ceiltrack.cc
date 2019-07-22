#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ceiltrack/ceiltrack.h"

// distance to closest grid point
static float griddist(float x, float q) {
  float r = remainderf(x + q / 2, q);
  if (r < 0) r += q;
  return r - q/2;
}

bool CeilingTracker::Open(const char *fname) {
  uint8_t header[20];
  FILE *fp = fopen(fname, "rb");
  if (!fp) {
    perror(fname);
    return false;
  }

  if (fread(header, 1, sizeof(header), fp) != sizeof(header)) {
    fprintf(stderr, "CeilingTracker::Open: short read\n");
    goto err;
  }

  if (header[0] != 'c' || header[1] != 'm' || header[2] != 'L' || header[3] != 'U') {
    fprintf(stderr, "CeilingTracker::Open: bad magic\n");
    goto err;
  }

  uint16_t h, w;
  uint32_t uvsiz, rlesiz;
  memcpy(&h, header+8, 2);
  memcpy(&w, header+10, 2);
  memcpy(&uvsiz, header+12, 4);
  memcpy(&rlesiz, header+16, 4);
  fprintf(stderr,
          "CeilingTracker::Open: %dx%d imgsiz, %d uv table entries, %d rle "
          "entries\n",
          w, h, uvsiz, rlesiz);

  mask_rle_ = new uint16_t[rlesiz];
  mask_rlelen_ = rlesiz;
  uvmap_ = new __fp16[uvsiz*2];
  if (fread(mask_rle_, 2, rlesiz, fp) != rlesiz) {
    fprintf(stderr, "CeilingTracker::Open: short read on rle table\n");
    goto err;
  }

  if (fread(uvmap_, 4, uvsiz, fp) != uvsiz) {
    fprintf(stderr, "CeilingTracker::Open: short read on uv table\n");
    goto err;
  }

  fclose(fp);
  return true;

err:
  fclose(fp);
  return false;
}

float CeilingTracker::Update(const uint8_t *img, uint8_t thresh, float xgrid,
                             float ygrid, float *xytheta) {
  int rleptr = 0;
  int uvptr = 0;

  // this is solvable in closed form! it's a pre-inverted 3x3 matrix * a 3x1 vector
  float u = xytheta[0], v = xytheta[1], theta = xytheta[2];
  float S = sin(theta), C = cos(theta);
  float S2 = 0, S3 = 0, R = 0;
  float Sdx = 0, Sdy = 0, SdRxy = 0;
  float cost = 0;
  int N = 0;

  while (rleptr < mask_rlelen_) {
    // read zero-len
    img += mask_rle_[rleptr++];
    uint16_t n = mask_rle_[rleptr++];
    while (n--) {
      if ((*img) > thresh) {
        float x = uvmap_[uvptr];
        float y = uvmap_[uvptr+1];
        R += x * x + y * y;
        float Rx = x * C + y * S, Ry = -x * S + y * C;
        float dRx = x * S - C * y, dRy = x * C + S * y;
        float dx = griddist(Rx - u, xgrid);
        float dy = griddist(Ry - v, ygrid);
        cost += dx*dx + dy*dy;
        S2 += dRx;
        S3 += dRy;
        Sdx += dx;
        Sdy += dy;
        SdRxy += dx*dRx + dy*dRy;
        N++;
      }
      img++;
      uvptr += 2;
    }
  }

  // Levenberg-Marquardt damping factor (if no detections, prevents blowups)
  const float lambda = 1;
  #if 0
  printf("JTJ | %f %f %f\n", N + lambda, 0.0f, S2);
  printf("    | %f %f %f\n", 0.0f, N + lambda, S3);
  printf("    | %f %f %f\n", S2, S3, R + lambda);
  printf("JTr | %f %f %f\n", -Sdx, -Sdy, -SdRxy);
  #endif
  {
    float x0 = S3 * Sdy;
    float x1 = N + lambda;
    float x2 = SdRxy * x1;
    float x3 = -x1 * (R + lambda);
    float x4 = S3 * S3 + x3;
    float x5 = S2 * S2;
    float x6 = 1.0 / (x4 + x5);
    float x7 = x6 / x1;
    float x8 = S2 * Sdx;
    xytheta[0] -= x7 * (S2 * (x0 - x2) - Sdx * x4);
    xytheta[1] -= x7 * (-S3 * x2 + S3 * x8 - Sdy * (x3 + x5));
    xytheta[2] -= x6 * (-x0 + x2 - x8);
  }

  return 0.5*cost;
}