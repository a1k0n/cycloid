#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "localization/coneslam/localize.h"

namespace coneslam {

// const float NOISE_ANGULAR = 0.4;
// const float NOISE_LONG = 16;
// const float NOISE_LAT = 8;

const float CONE_RADIUS = 44.5/M_PI/200.;
const float NOISE_ANGULAR = 0.8*3.3;
const float NOISE_LONG = 8*3.3;
const float NOISE_LAT = 8*3.3;
const float NOISE_STEER_u = 0.3*3.3;
const float NOISE_STEER_s = 0.3*3.3;

static double randn() {
  // #include <random> doesn't work in my ARM cross-compiler so I'm just
  // doing something dumb here
  // it's slightly heavier-tailed than a gaussian and cuts off at -6..6, but
  // that's OK
  double n = drand48();
  for (int i = 1; i < 6; i++) n += drand48();
  return 2*n - 6;
}

Localizer::~Localizer() {
  delete[] particles_;
  delete[] landmarks_;
  delete[] LL_;
  delete[] c0_;
  delete[] c1_;
}

void Localizer::Reset() {
  for (int i = 0; i < n_particles_; i++) {
    particles_[i].x = 0.025*randn() + home_x_;
    particles_[i].y = 0.025*randn() + home_y_;
    particles_[i].theta = randn() * 0.1 + home_theta_;
    particles_[i].heading = particles_[i].theta;
  }
  ResetLikelihood();
}

void Localizer::ResetLikelihood() {
  for (int i = 0; i < n_particles_; i++) {
    LL_[i] = -1e6;
  }
  LLmax_ = -1e6;
}

bool Localizer::LoadLandmarks(const char *filename) {
  n_landmarks_ = 0;

  FILE *fp = fopen(filename, "r");
  if (!fp) {
    perror(filename);
    return false;
  }

  if (fscanf(fp, "%d\n", &n_landmarks_) != 1) {
    fprintf(stderr, "unable to read number of landmarks from %s\n", filename);
    fclose(fp);
    return false;
  }
  landmarks_ = new Landmark[n_landmarks_];
  for (int i = 0; i < n_landmarks_; i++) {
    if (fscanf(fp, "%f %f\n", &landmarks_[i].x, &landmarks_[i].y) != 2) {
      fprintf(stderr, "unable to read landmark #%d from %s\n", i, filename);
      fclose(fp);
      return false;
    }
  }
  // if the last line is "home [x] [y] [theta]", override the home location
  if (fscanf(fp, "home %f %f %f\n", &home_x_, &home_y_, &home_theta_) == 3) {
    fprintf(stderr, "home location set to %f %f %f\n",
        home_x_, home_y_, home_theta_);
  }
  fclose(fp);
  c0_ = new uint16_t[n_particles_ * n_landmarks_];
  c1_ = new uint16_t[n_particles_ * n_landmarks_];
  Reset();
  return true;
}

void Localizer::Predict(float ds, float w, float dt) {
  for (int i = 0; i < n_particles_; i++) {
    float t = particles_[i].theta + w*dt + randn()*NOISE_ANGULAR*ds*dt;

    // low-pass filter the forward direction to determine the car's travel
    // direction (heading); this way we spread out the particles in a turn
    // assuming some unknown amount of understeer
    float alpha = randn() * NOISE_STEER_s + NOISE_STEER_u;
    float h = particles_[i].heading;
    h += alpha*(t - h);

    // disable for now
    h = t;

    float S = sin(h);
    float C = cos(h);

    float dx = ds + randn()*NOISE_LONG*ds*dt;
    float dy = randn()*NOISE_LAT*ds*dt;

    particles_[i].x += dx*C - dy*S;
    particles_[i].y += dx*S + dy*C;
    particles_[i].theta = t;
    particles_[i].heading = h;
  }
}

void Localizer::UpdateLM(float lm_bearing, float precision,
                         float bogon_thresh) {
  float mindiffsqr = bogon_thresh*bogon_thresh;

  // for each particle, find likeliest landmark and its likelihood
  for (int i = 0; i < n_particles_; i++) {
    const Particle &p = particles_[i];
    float S = sin(p.theta),
          C = cos(p.theta);
#ifdef PF_DEBUG
    printf("%d: ", i);
#endif
    for (int j = 0; j < n_landmarks_; j++) {
      const Landmark &l = landmarks_[j];
      float dx = l.x - p.x,
            dy = l.y - p.y;
      float z = dx*C + dy*S,
            y = dx*S - dy*C;
      float d = sqrt(dx*dx + dy*dy);
      float coneangle = 2*asin(fmin(CONE_RADIUS / d, 1));
      float diff = fmax(fabs(atan2f(y, z) - lm_bearing) - coneangle, 0);
      float L = -precision*fmin(mindiffsqr, diff*diff);
#ifdef PF_DEBUG
      printf("[%d]%f %f ", j, diff, L);
#endif
      if (L > LL_[i]) {
        LL_[i] = L;
      }
    }
#ifdef PF_DEBUG
    printf("LL[i]=%f\n", LL_[i]);
#endif
    if (LL_[i] > LLmax_) {
      LLmax_ = LL_[i];
    }
  }
#ifdef PF_DEBUG
  printf("LLmax=%f (%d landmarks)\n", LLmax_, n_landmarks_);
#endif
}

// FIXME: this is all hardcoded from OpenCV until we can integrate with the
// fisheye class and just load the calibration params off disk
static const int16_t fisheyeLUT[] = {
#include "uvmap1.txt"
};

void Localizer::Update(const uint8_t *yuvimg, float temperature) {
  const uint8_t *V = yuvimg+(640*480 + 320*240);
  memset(activations_, 0, sizeof(activations_));
  int idx = 0;
  // remap fisheye into an array of pixel activations
  for (int j = 0; j < kFisheyeLUT_h; j++) {
    for (int i = 0; i < kFisheyeLUT_w; i++) {
      int16_t x = fisheyeLUT[idx++];
      int16_t y = fisheyeLUT[idx++];
      if (x < 0 || x >= 320 || y < 0 || y >= 240) continue;
      // an activation is just the signed V channel magnitude
      activations_[i] += static_cast<int32_t>(V[y*320 + x]) - 128;
    }
  }
  // make a second copy of the activations to handle angular wraparound
  for (int i = 0; i < kFisheyeLUT_w; i++) {
    activations_[kFisheyeLUT_w + i] = activations_[i];
  }
  // now do a cumulative sum
  for (int i = 1; i < 2 * kFisheyeLUT_w; i++) {
    activations_[i] += activations_[i-1];
  }

  const float angratio = kFisheyeLUT_w / (2*M_PI);
  // next, get the likelihood of each particle by looking up the expected
  // position of each cone and adding up the summed activations
  int c0idx = 0;
  for (int i = 0; i < n_particles_; i++) {
    const Particle &p = particles_[i];
    float S = sin(p.theta),
          C = cos(p.theta);

    LL_[i] = 0;
    for (int j = 0; j < n_landmarks_; j++) {
      const Landmark &l = landmarks_[j];
      float dx = l.x - p.x,
            dy = l.y - p.y;
      float z = dx*C + dy*S,
            y = -dx*S + dy*C;
      float d = sqrt(dx*dx + dy*dy);
      float visibleradius = angratio * asin(fmin(CONE_RADIUS / d, 1));
      float coneangle = angratio * atan2f(y, z);
      int c0 = roundf(coneangle - visibleradius);
      int c1 = roundf(coneangle + visibleradius);
      if (c0 < 0) {
        c0 += kFisheyeLUT_w;
        c1 += kFisheyeLUT_w;
      }
      assert(c0 >= 0);
      assert(c0 < kFisheyeLUT_w*2);
      assert(c1 >= 0);
      assert(c1 < kFisheyeLUT_w*2);
      c0_[c0idx] = c0;
      c1_[c0idx] = c1;
      c0idx++;
      LL_[i] += (activations_[c1] - activations_[c0]) * temperature;
    }
    if (LL_[i] > LLmax_) {
      LLmax_ = LL_[i];
    }
  }
}

void Localizer::Resample() {
  // now, normalize the distribution and resample particles
  float totalP = 0;
  for (int i = 0; i < n_particles_; i++) {
    LL_[i] = exp(LL_[i] - LLmax_);
    totalP += LL_[i];
#ifdef PF_DEBUG
    printf("%0.3f ", LL_[i]);
#endif
  }
#ifdef PF_DEBUG
  printf(" | total=%f\nresample: ", totalP);
#endif
  float deltaP = totalP / n_particles_;
  // pick a random starting location weighted by particle likelihood
  float randP = drand48() * totalP;
  Particle *newp = new Particle[n_particles_];
  int j = 0;
  for (int i = 0; i < n_particles_; i++) {
    while (randP > LL_[j]) {
      randP -= LL_[j];
      j++;
      if (j == n_particles_) {
        j = 0;
      }
    }
    newp[i] = particles_[j];

    // canonicalize angles while resampling
    // newp[i].theta = fmodf(newp[i].theta + M_PI, 2*M_PI) - M_PI;
    // NO! we can't do this if we then go around and average theta!

#ifdef PF_DEBUG
    printf("%d ", j);
#endif
    randP += deltaP;
  }
#ifdef PF_DEBUG
  printf("\n");
#endif

  ResetLikelihood();

  delete[] particles_;
  particles_ = newp;
}

bool Localizer::GetLocationEstimate(Particle *mean) const {
  mean->x = 0;
  mean->y = 0;
  // FIXME: the theta and heading estimates are meaningless here; we really
  // need to average sin/cosine and atan2 to find the result
  mean->theta = 0;
  mean->heading = 0;
  for (int i = 0; i < n_particles_; i++) {
    mean->x += particles_[i].x;
    mean->y += particles_[i].y;
    mean->theta += particles_[i].theta;
    mean->heading += particles_[i].heading;
  }
  mean->x /= n_particles_;
  mean->y /= n_particles_;
  mean->theta /= n_particles_;
  mean->heading /= n_particles_;
  return true;
}

int Localizer::SerializedSize() const {
  // Saves two chunks out: particle positions and activations
  // we should also show the particle lookups done...?
  int len = 8 + n_particles_ * sizeof(Particle);  // IFF header + particles
  len += 8 + kFisheyeLUT_w * sizeof(int32_t);  // IFF header + activations

  // IFF header + #landmarks + per-particle expectations
  len += 8 + 1 + 2 * n_landmarks_ * n_particles_ * sizeof(uint16_t);
  return len;
}

int Localizer::Serialize(uint8_t *buf, int buflen) const {
  int totallen = SerializedSize();
  assert(buflen > totallen);
  uint32_t len = n_particles_ * sizeof(Particle) + 8;
  memcpy(buf, "MCL4", 4);  // monte carlo localizer, 4-dim particles
  memcpy(buf+4, &len, 4);
  memcpy(buf+8, particles_, n_particles_ * sizeof(Particle));
  buf += 8 + n_particles_ * sizeof(Particle);

  len = kFisheyeLUT_w * sizeof(int32_t) + 8;
  memcpy(buf, "aCDF", 4);  // activation cumulative distribution function
  memcpy(buf+4, &len, 4);
  memcpy(buf+8, activations_, kFisheyeLUT_w * sizeof(int32_t));
  buf += 8 + kFisheyeLUT_w * sizeof(int32_t);

  len = 8 + 1 + 2 * n_landmarks_ * n_particles_* sizeof(uint16_t);
  memcpy(buf, "LM01", 4);  // landmark location
  memcpy(buf+4, &len, 4);
  buf[8] = n_landmarks_;
  buf += 9;
  memcpy(buf, c0_, n_landmarks_ * n_particles_ * sizeof(uint16_t));
  buf += n_landmarks_ * n_particles_ * 2;
  memcpy(buf, c1_, n_landmarks_ * n_particles_ * sizeof(uint16_t));
  buf += n_landmarks_ * n_particles_ * 2;

  return totallen;
}

int Localizer::HeaderSize() const {
  return 8 + n_landmarks_ * sizeof(Landmark);
}

int Localizer::SerializeHeader(uint8_t *buf, int buflen) const {
  int totallen = HeaderSize();
  assert(buflen > totallen);
  memcpy(buf, "CONE", 4);  // cone locations
  memcpy(buf+4, &totallen, 4);
  memcpy(buf+8, landmarks_, n_landmarks_ * sizeof(Landmark));
  return totallen;
}

}  // namespace coneslam
