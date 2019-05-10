#include <stdio.h>
#include <stdint.h>

#include "drive/vflookup.h"

bool ValueFuncLookup::Init() {
  FILE *fp = fopen("vf.bin", "rb");
  if (!fp) {
    return false;
  }
  uint8_t hdr[8];
  if (fread(hdr, 1, 8, fp) != 8) {
    goto bad;
  }
  if (hdr[0] != 'V' || hdr[1] != 'F' || hdr[2] != 'N' || hdr[3] != 'C')
    goto bad;
  if (hdr[4] != 0x0a) goto bad;
  uint16_t a, h, w;
  fread(&a, 2, 1, fp);
  fread(&h, 2, 1, fp);
  fread(&w, 2, 1, fp);
  fread(&scale_, 4, 1, fp);
  a_ = a;
  h_ = h;
  w_ = w;
  data_ = new __fp16[a*h*w];
  fread(data_, a*h*w, 2, fp);
  fclose(fp);
  {
    float d1 = data_[0], d2 = data_[1], d3 = data_[2], d4 = data_[3];
    fprintf(stderr,
            "loaded vf.bin %dx%dx%d @ %f scale; first values are %f %f %f %f\n",
            a_, h_, w_, scale_, d1, d2, d3, d4);
  }
  return true;
bad:
  perror("invalid vf.bin");
  fclose(fp);
  return false;
}

ValueFuncLookup::~ValueFuncLookup() {
  delete[] data_;
}
