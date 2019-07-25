#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "ceiltrack/ceiltrack.h"

#if (defined __ARM_NEON) || (defined __ARM_NEON__)
#include <arm_neon.h>
#else
#include <emmintrin.h>
#include <pmmintrin.h>
#include <xmmintrin.h>
#endif

static inline float moddist(float x, float q, float ooq) {
  float xoq = x * ooq;
  return q*(xoq - __builtin_roundf(xoq));
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
  uvmaplen_ = uvsiz;
  if (fread(mask_rle_, 2, rlesiz, fp) != rlesiz) {
    fprintf(stderr, "CeilingTracker::Open: short read on rle table\n");
    goto err;
  }

  if (fread(uvmap_, 4, uvsiz, fp) != uvsiz) {
    fprintf(stderr, "CeilingTracker::Open: short read on uv table\n");
    goto err;
  }

#if 0
  for (float f = -10; f < 10; f += 0.125f) {
    printf("moddist(%f, 3.0) = %f\n", f, moddist(f, 3.0, 1.0/3.0));
  }
#endif

  fclose(fp);
  return true;

err:
  fclose(fp);
  return false;
}

#if (defined __ARM_NEON) || (defined __ARM_NEON__)
// vectorize w/ fp16x8!
float CeilingTracker::Update(const uint8_t *img, uint8_t thresh, float xgrid,
                             float ygrid, float *xytheta) {
  int rleptr = 0;
  int uvptr = 0;

  float ooxg = 1.0 / xgrid, ooyg = 1.0 / ygrid;

  // this is solvable in closed form! it's a pre-inverted 3x3 matrix * a 3x1
  // vector
  float u = xytheta[0], v = xytheta[1], theta = xytheta[2];
  float S = sin(theta), C = cos(theta);
  float S2 = 0, S3 = 0, R = 0;
  float Sdx = 0, Sdy = 0, SdRxy = 0;
  float cost = 0;
  int N = 0;

  static std::vector<float> xybuf;
  while (rleptr < mask_rlelen_) {
    // read zero-len
    img += mask_rle_[rleptr++];
    int n = mask_rle_[rleptr++];
    xybuf.clear();
    while (n--) {
      uvptr += 2;
      if ((*img++) <= thresh) {
        continue;
      }
      xybuf.push_back(uvmap_[uvptr - 2]);
      xybuf.push_back(uvmap_[uvptr - 1]);
    }
    n = xybuf.size();
    N += n/2;
    int i;
    for (i = 0; i < (n & ~3); i += 4) {
      float32x4_t xy2 = vld1q_f32(&xybuf[i]);
      
    }
    for (; i < n; i += 2) {
      float x = xybuf[i];
      float y = xybuf[i+1];
      R += x * x + y * y;
      float Rx = x * C + y * S, Ry = -x * S + y * C;
      float dRx = x * S - C * y, dRy = x * C + S * y;
      S2 += dRx;
      S3 += dRy;
      float dx = moddist(Rx - u, xgrid, ooxg);
      float dy = moddist(Ry - v, ygrid, ooyg);
      cost += dx * dx + dy * dy;
      Sdx += dx;
      Sdy += dy;
      SdRxy += dx * dRx + dy * dRy;
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

  return 0.5 * cost;
}

#else

float hsum_ps_sse3(__m128 v) {
  __m128 shuf = _mm_movehdup_ps(v);  // broadcast elements 3,1 to 2,0
  __m128 sums = _mm_add_ps(v, shuf);
  shuf = _mm_movehl_ps(shuf, sums);  // high half -> low half
  sums = _mm_add_ss(sums, shuf);
  return _mm_cvtss_f32(sums);
}

float CeilingTracker::Update(const uint8_t *img, uint8_t thresh, float xgrid,
                             float ygrid, float *xytheta, int niter,
                             bool verbose) {
  int rleptr = 0;
  int uvptr = 0;

  float ooxg = 1.0 / xgrid, ooyg = 1.0 / ygrid;

  // first step: lookup all the camera ray vectors of white pixels looking up
  static float *xybuf = NULL;
  int bufptr = 0;
  if (xybuf == NULL) {
    // needs to have 16-byte alignment, which it should, being a relatively
    // large allocation
    xybuf = new float[uvmaplen_];
  }
  while (rleptr < mask_rlelen_) {
    // read zero-len
    img += mask_rle_[rleptr++];
    int n = mask_rle_[rleptr++];
    while (n--) {
      if ((*img++) > thresh) {
        xybuf[bufptr] = uvmap_[uvptr];
        xybuf[bufptr + 1] = uvmap_[uvptr + 1];
        bufptr += 2;
      }
      uvptr += 2;
    }
  }

  float cost = 0;
  for (int iter = 0; iter < niter; iter++) {
    // this is solvable in closed form! it's a pre-inverted 3x3 matrix * a 3x1
    // vector
    float u = xytheta[0], v = xytheta[1], theta = xytheta[2];
    float S = sin(theta), C = cos(theta);

    // vectorized, 4 pixels at a time
    _MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);
    __m128 Cvec = _mm_set1_ps(C);
    __m128 Svec = _mm_set1_ps(S);
    __m128 Rvec = _mm_setzero_ps();
    __m128 S2vec = _mm_setzero_ps();
    __m128 S3vec = _mm_setzero_ps();
    __m128 SdRxyvec = _mm_setzero_ps();
    __m128 Sdxvec = _mm_setzero_ps();
    __m128 Sdyvec = _mm_setzero_ps();
    __m128 costvec = _mm_setzero_ps();
    int M = bufptr & (~7);
    for (int i = 0; i < M; i += 8) {
      __m128 xyxy1 = _mm_load_ps(&xybuf[i]);
      __m128 xyxy2 = _mm_load_ps(&xybuf[i + 4]);
      __m128 xxxx = _mm_shuffle_ps(xyxy1, xyxy2, _MM_SHUFFLE(2, 0, 2, 0));
      __m128 yyyy = _mm_shuffle_ps(xyxy1, xyxy2, _MM_SHUFFLE(3, 1, 3, 1));
      Rvec = _mm_add_ps(
          Rvec, _mm_add_ps(_mm_mul_ps(xxxx, xxxx), _mm_mul_ps(yyyy, yyyy)));
      __m128 Rxxxx = _mm_add_ps(_mm_mul_ps(xxxx, Cvec), _mm_mul_ps(yyyy, Svec));
      __m128 Ryyyy = _mm_sub_ps(_mm_mul_ps(yyyy, Cvec), _mm_mul_ps(xxxx, Svec));
      S2vec = _mm_sub_ps(S2vec, Ryyyy);
      S3vec = _mm_add_ps(S3vec, Rxxxx);
      __m128 Rxoq =
          _mm_mul_ps(_mm_sub_ps(Rxxxx, _mm_set1_ps(u)), _mm_set1_ps(ooxg));
      __m128 Ryoq =
          _mm_mul_ps(_mm_sub_ps(Ryyyy, _mm_set1_ps(v)), _mm_set1_ps(ooyg));
      __m128 Rxrounded = _mm_cvtepi32_ps(_mm_cvtps_epi32(Rxoq));
      __m128 Ryrounded = _mm_cvtepi32_ps(_mm_cvtps_epi32(Ryoq));
      __m128 dxxxx =
          _mm_mul_ps(_mm_sub_ps(Rxoq, Rxrounded), _mm_set1_ps(xgrid));
      __m128 dyyyy =
          _mm_mul_ps(_mm_sub_ps(Ryoq, Ryrounded), _mm_set1_ps(ygrid));
      Sdxvec = _mm_add_ps(Sdxvec, dxxxx);
      Sdyvec = _mm_add_ps(Sdyvec, dyyyy);
      costvec = _mm_add_ps(costvec, _mm_add_ps(_mm_mul_ps(dxxxx, dxxxx),
                                               _mm_mul_ps(dyyyy, dyyyy)));
      SdRxyvec = _mm_add_ps(SdRxyvec, _mm_sub_ps(_mm_mul_ps(Rxxxx, dyyyy),
                                                 _mm_mul_ps(Ryyyy, dxxxx)));
    }

    // Levenberg-Marquardt damping factor (if no detections, prevents blowups)
    const float lambda = 1;
    float N = M >> 1;
    float R = hsum_ps_sse3(Rvec);
    float cost = hsum_ps_sse3(costvec);
    float S2 = hsum_ps_sse3(S2vec);
    float S3 = hsum_ps_sse3(S3vec);
    float Sdx = hsum_ps_sse3(Sdxvec);
    float Sdy = hsum_ps_sse3(Sdyvec);
    float SdRxy = hsum_ps_sse3(SdRxyvec);
#if 0
  printf("sse: R=%f cost=%f\n", R, cost);
  printf("JTJ | %f %f %f\n", N + lambda, 0.0f, S2);
  printf("    | %f %f %f\n", 0.0f, N + lambda, S3);
  printf("    | %f %f %f\n", S2, S3, R + lambda);
  printf("JTr | %f %f %f\n", -Sdx, -Sdy, -SdRxy);
#endif

#if 0
    // unvectorized remainder
    for (; i < M; i += 2) {
      float x = xybuf[i];
      float y = xybuf[i + 1];
      float Rx = x * C + y * S, Ry = -x * S + y * C;
      R += x * x + y * y;
      S2 -= Ry;
      S3 += Rx;
      float dx = moddist(Rx - u, xgrid, ooxg);
      float dy = moddist(Ry - v, ygrid, ooyg);
      cost += dx * dx + dy * dy;
      Sdx += dx;
      Sdy += dy;
      SdRxy += -dx * Ry + dy * Rx;
    }

#if 0
  printf("JTJ | %f %f %f\n", N + lambda, 0.0f, S2);
  printf("    | %f %f %f\n", 0.0f, N + lambda, S3);
  printf("    | %f %f %f\n", S2, S3, R + lambda);
  printf("JTr | %f %f %f\n", -Sdx, -Sdy, -SdRxy);
#endif
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

    if (verbose) {
      printf("CeilTrack::Update iter %d: cost %f xyt %f %f %f (%d pixels)\n",
             iter, cost * 0.5, xytheta[0], xytheta[1], xytheta[2], M / 2);
    }
  }

  return 0.5 * cost;
}

#endif