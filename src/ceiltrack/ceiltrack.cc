#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "ceiltrack/ceiltrack.h"

#if (defined __ARM_NEON) || (defined __ARM_NEON__)
#include <arm_neon.h>
#elif defined __SSE3__  // you'll probably need -msse3 to enable this
#include <emmintrin.h>
#include <immintrin.h>
#include <pmmintrin.h>
#include <xmmintrin.h>
#endif

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

  if (header[0] != 'c' || header[1] != 'm' || header[2] != 'L' ||
      header[3] != 'U') {
    fprintf(stderr, "CeilingTracker::Open: bad magic\n");
    goto err;
  }

  uint16_t h, w;
  uint32_t uvsiz, rlesiz;
  memcpy(&h, header + 8, 2);
  memcpy(&w, header + 10, 2);
  memcpy(&uvsiz, header + 12, 4);
  memcpy(&rlesiz, header + 16, 4);
  fprintf(stderr,
          "CeilingTracker::Open: %dx%d imgsiz, %d uv table entries, %d rle "
          "entries\n",
          w, h, uvsiz, rlesiz);

  mask_rle_ = new uint16_t[rlesiz];
  mask_rlelen_ = rlesiz;
  uvmap_ = new uint32_t[uvsiz];
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

float hsum_f32_neon(float32x4_t x) {
  float32x2_t r2 = vpadd_f32(vget_high_f32(x), vget_low_f32(x));
  return vget_lane_f32(vpadd_f32(r2, r2), 0);
}

// vectorize w/ fp16x8!
float CeilingTracker::Update(const uint8_t *img, uint8_t thresh, float xgrid,
                             float ygrid, float *xytheta, int niter,
                             bool verbose) {
  int rleptr = 0;
  int uvptr = 0;

  float ooxg = 1.0 / xgrid, ooyg = 1.0 / ygrid;

  // first step: lookup all the camera ray vectors of white pixels looking up
  static uint32_t *xybuf = NULL;
  int bufptr = 0;
  if (xybuf == NULL) {
    // needs to have 16-byte alignment, which it should, being a relatively
    // large allocation
    xybuf = new uint32_t[uvmaplen_];
  }
  while (rleptr < mask_rlelen_) {
    // read zero-len
    img += mask_rle_[rleptr++];
    int n = mask_rle_[rleptr++];
    while (n--) {
      if ((*img++) > thresh) {
        xybuf[bufptr++] = uvmap_[uvptr];
      }
      uvptr++;
    }
  }

  float cost = 0;
  for (int iter = 0; iter < niter; iter++) {
    // this is solvable in closed form! it's a pre-inverted 3x3 matrix * a 3x1
    // vector
    float u = xytheta[0], v = xytheta[1], theta = xytheta[2];
    float S = sin(theta), C = cos(theta);

    float32x4_t S2vec = vmovq_n_f32(0), S3vec = vmovq_n_f32(0),
                Rvec = vmovq_n_f32(0), costvec = vmovq_n_f32(0),
                SdRxyvec = vmovq_n_f32(0), Sdxvec = vmovq_n_f32(0),
                Sdyvec = vmovq_n_f32(0);
    float32x4_t Cvec = vld1q_dup_f32(&C);
    float32x4_t Svec = vld1q_dup_f32(&S);

    int M = bufptr & (~3);
    for (int i = 0; i < M; i += 4) {
      // load four interleaved coordinates, cvt to 32-bit floats
      float16x4x2_t xxxxyyyy = vld2_f16((const __fp16*)&xybuf[i]);
      float32x4_t xxxx = vcvt_f32_f16(xxxxyyyy.val[0]);
      float32x4_t yyyy = vcvt_f32_f16(xxxxyyyy.val[1]);

      Rvec = vaddq_f32(Rvec,
                       vaddq_f32(vmulq_f32(xxxx, xxxx), vmulq_f32(yyyy, yyyy)));

      float32x4_t Rxxxx =
          vaddq_f32(vmulq_f32(xxxx, Cvec), vmulq_f32(yyyy, Svec));
      float32x4_t Ryyyy =
          vsubq_f32(vmulq_f32(yyyy, Cvec), vmulq_f32(xxxx, Svec));

      S2vec = vsubq_f32(S2vec, Ryyyy);
      S3vec = vaddq_f32(S3vec, Rxxxx);
      float32x4_t Rxoq =
          vmulq_f32(vsubq_f32(Rxxxx, vld1q_dup_f32(&u)), vld1q_dup_f32(&ooxg));
      float32x4_t Ryoq =
          vmulq_f32(vsubq_f32(Ryyyy, vld1q_dup_f32(&v)), vld1q_dup_f32(&ooyg));
      float32x4_t Rxoqp5 = vaddq_f32(Rxoq, vmovq_n_f32(0.5));
      float32x4_t Ryoqp5 = vaddq_f32(Ryoq, vmovq_n_f32(0.5));
      // NEON only supports truncating toward zero but we need to round to
      // nearest so we do a trick here: first we use compare w/ 0 and
      // reinterpret the bit pattern so that if element i was negative, negx[i]
      // = -1. then we add 0.5, truncate, and add negx which will have the
      // effect of adding -0.5 if the original number was negative, which
      // achieves correct rounding.
      int32x4_t negx = vreinterpretq_s32_u32(vcltq_f32(Rxoqp5, vmovq_n_f32(0)));
      int32x4_t negy = vreinterpretq_s32_u32(vcltq_f32(Ryoqp5, vmovq_n_f32(0)));
      float32x4_t Rxrounded =
          vcvtq_f32_s32(vaddq_s32(negx, vcvtq_s32_f32(Rxoqp5)));
      float32x4_t Ryrounded =
          vcvtq_f32_s32(vaddq_s32(negy, vcvtq_s32_f32(Ryoqp5)));

      float32x4_t dxxxx =
          vmulq_f32(vsubq_f32(Rxoq, Rxrounded), vld1q_dup_f32(&xgrid));
      float32x4_t dyyyy =
          vmulq_f32(vsubq_f32(Ryoq, Ryrounded), vld1q_dup_f32(&ygrid));
#if 0
      float tx[4], ty[4], txr[4], tyr[4];
      float tdx[4], tdy[4];
      vst1q_f32(tx, Rxoq);
      vst1q_f32(txr, Rxrounded);
      vst1q_f32(ty, Ryoq);
      vst1q_f32(tyr, Ryrounded);
      vst1q_f32(tdx, dxxxx);
      vst1q_f32(tdy, dyyyy);
      for (int j = 0; j < 4; j++) {
        float x = xybuf[i + j * 2];
        float y = xybuf[i + j * 2 + 1];
        float Rx = x * C + y * S, Ry = -x * S + y * C;
        float dx = moddist(Rx - u, xgrid, ooxg);
        float dy = moddist(Ry - v, ygrid, ooyg);
        printf("neon%d dx: %f %f %f %f plain: %f %f %f %f\n", j, tx[j], ty[j],
               txr[j], tyr[j], (Rx - u) * ooxg, roundf((Rx - u) * ooxg),
               (Ry - v) * ooyg, roundf((Ry - u) * ooyg));
        printf("  %f %f -> %f %f\n", tdx[j], tdy[j], dx, dy);
      }

      return 0;
#endif

      Sdxvec = vaddq_f32(Sdxvec, dxxxx);
      Sdyvec = vaddq_f32(Sdyvec, dyyyy);
      costvec = vaddq_f32(
          costvec, vaddq_f32(vmulq_f32(dxxxx, dxxxx), vmulq_f32(dyyyy, dyyyy)));
      SdRxyvec = vaddq_f32(SdRxyvec, vsubq_f32(vmulq_f32(Rxxxx, dyyyy),
                                               vmulq_f32(Ryyyy, dxxxx)));
    }

    float N = M;
    float R = hsum_f32_neon(Rvec);
    cost = hsum_f32_neon(costvec);
    float S2 = hsum_f32_neon(S2vec);
    float S3 = hsum_f32_neon(S3vec);
    float Sdx = hsum_f32_neon(Sdxvec);
    float Sdy = hsum_f32_neon(Sdyvec);
    float SdRxy = hsum_f32_neon(SdRxyvec);
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

    if (verbose) {
      printf("CeilTrack::Update iter %d: cost %f xyt %f %f %f (%d pixels)\n",
             iter, cost * 0.5, xytheta[0], xytheta[1], xytheta[2], M / 2);
    }
  }

  return 0.5 * cost;
}

#elif defined __SSE3__

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
  static uint32_t *xybuf = NULL;
  int bufptr = 0;
  if (xybuf == NULL) {
    // needs to have 16-byte alignment, which it should, being a relatively
    // large allocation
    xybuf = new uint32_t[uvmaplen_];
  }
  while (rleptr < mask_rlelen_) {
    // read zero-len
    img += mask_rle_[rleptr++];
    int n = mask_rle_[rleptr++];
    while (n--) {
      if ((*img++) > thresh) {
        xybuf[bufptr++] = uvmap_[uvptr];
      }
      uvptr++;
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
    int M = bufptr & (~3);
    for (int i = 0; i < M; i += 4) {
      // convert from fp16 to fp32
      __m128i xyfp16x8 = _mm_loadu_si128((__m128i const*) (xybuf+i));
      __m128 xyxy1 = _mm_cvtph_ps(xyfp16x8);
      __m128 xyxy2 = _mm_cvtph_ps(_mm_unpackhi_epi64(xyfp16x8, xyfp16x8));
      __m128 xxxx = _mm_shuffle_ps(xyxy1, xyxy2, _MM_SHUFFLE(2, 0, 2, 0));
      __m128 yyyy = _mm_shuffle_ps(xyxy1, xyxy2, _MM_SHUFFLE(3, 1, 3, 1));
      #if 0
      {
        float t1[4], t2[4];
        _mm_store_ps(t1, xxxx);
        _mm_store_ps(t2, yyyy);
        for (int j = 0; j < 4; j++) {
          printf("%f %f\n", t1[j], t2[j]);
        }
        exit(0);
      }
      #endif
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
    float N = M;
    float R = hsum_ps_sse3(Rvec);
    cost = hsum_ps_sse3(costvec);
    float S2 = hsum_ps_sse3(S2vec);
    float S3 = hsum_ps_sse3(S3vec);
    float Sdx = hsum_ps_sse3(Sdxvec);
    float Sdy = hsum_ps_sse3(Sdyvec);
    float SdRxy = hsum_ps_sse3(SdRxyvec);
#if 0
  if (verbose) {
    printf("sse: R=%f cost=%f\n", R, cost);
    printf("JTJ | %f %f %f\n", N + lambda, 0.0f, S2);
    printf("    | %f %f %f\n", 0.0f, N + lambda, S3);
    printf("    | %f %f %f\n", S2, S3, R + lambda);
    printf("JTr | %f %f %f\n", -Sdx, -Sdy, -SdRxy);
  }
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
             iter, cost * 0.5, xytheta[0], xytheta[1], xytheta[2], M);
    }
  }

  return 0.5 * cost;
}

#else  // plain ol' unvectorized float version

static inline float moddist(float x, float q, float ooq) {
  float xoq = x * ooq;
  // hack: avoid extra work doing directional rounding by just adding 1024
  return q * (xoq - ((int)(xoq+1024.5f)) + 1024.f);
}

static inline float half_to_float_fast5(uint16_t h) {
  typedef union {
    uint32_t u;
    float f;
  } FP32;
  static const FP32 magic = {(254 - 15) << 23};
  FP32 o;

  o.u = (h & 0x7fff) << 13;  // exponent/mantissa bits
  o.f *= magic.f;            // exponent adjust
  o.u |= (h & 0x8000) << 16;  // sign bit
  return o.f;
}

float CeilingTracker::Update(const uint8_t *img, uint8_t thresh, float xgrid,
                             float ygrid, float *xytheta, int niter,
                             bool verbose) {
  int rleptr = 0;
  int uvptr = 0;

  float ooxg = 1.0 / xgrid, ooyg = 1.0 / ygrid;

  // first step: lookup all the camera ray vectors of white pixels looking up
  static uint32_t *xybuf = NULL;
  int bufptr = 0;
  if (xybuf == NULL) {
    // needs to have 16-byte alignment, which it should, being a relatively
    // large allocation
    xybuf = new uint32_t[uvmaplen_];
  }
  while (rleptr < mask_rlelen_) {
    // read zero-len
    img += mask_rle_[rleptr++];
    int n = mask_rle_[rleptr++];
    while (n--) {
      if ((*img++) > thresh) {
        xybuf[bufptr++] = uvmap_[uvptr];
      }
      uvptr++;
    }
  }

  float cost = 0;
  for (int iter = 0; iter < niter; iter++) {
    // this is solvable in closed form! it's a pre-inverted 3x3 matrix * a 3x1
    // vector
    float u = xytheta[0], v = xytheta[1], theta = xytheta[2];
    float S = sin(theta), C = cos(theta);

    cost = 0;
    float R = 0;
    float S2 = 0;
    float S3 = 0;
    float Sdx = 0;
    float Sdy = 0;
    float SdRxy = 0;

    // unvectorized remainder
    for (int i = 0; i < bufptr; i++) {
      float x = half_to_float_fast5(*((uint16_t *)(xybuf + i)));
      float y = half_to_float_fast5(*((uint16_t *)(xybuf + i) + 1));
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

    const float lambda = 1;
    float N = bufptr;
#if 0
    if (verbose) {
      printf("JTJ | %f %f %f\n", N + lambda, 0.0f, S2);
      printf("    | %f %f %f\n", 0.0f, N + lambda, S3);
      printf("    | %f %f %f\n", S2, S3, R + lambda);
      printf("JTr | %f %f %f\n", -Sdx, -Sdy, -SdRxy);
    }
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
             iter, cost * 0.5, xytheta[0], xytheta[1], xytheta[2], bufptr);
    }
  }

  return 0.5 * cost;
}

#endif
