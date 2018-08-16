#include <math.h>
#include <stdio.h>

#include "coneslam/localize.h"

namespace coneslam {

const float NOISE_ANGULAR = 0.4;
const float NOISE_LONG = 20;
const float NOISE_LAT = 1;
const float LM_SELECTIVITY = 20;

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
}

void Localizer::Reset() {
  for (int i = 0; i < n_particles_; i++) {
    particles_[i].x = randn();
    particles_[i].y = randn();
    particles_[i].theta = randn() * 0.5;
  }
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
  fclose(fp);
  return true;
}

void Localizer::Predict(float ds, float w, float dt) {
  for (int i = 0; i < n_particles_; i++) {
    float t = particles_[i].theta + w*dt + randn()*NOISE_ANGULAR*dt;
    float S = sin((particles_[i].theta + t)*0.5);
    float C = cos((particles_[i].theta + t)*0.5);

    float dx = ds + randn()*NOISE_LONG*dt;
    float dy = randn()*NOISE_LAT*dt;

    particles_[i].x += dx*C - dy*S;
    particles_[i].y += dx*S + dy*C;
    particles_[i].theta = t;
  }
}

void Localizer::UpdateLM(float lm_bearing) {
  float *LL;
  LL = new float[n_particles_];
  float LLmax = -1e6;

  // for each particle, find likeliest landmark and its likelihood
  for (int i = 0; i < n_particles_; i++) {
    const Particle &p = particles_[i];
    float S = sin(p.theta),
          C = cos(p.theta);
    LL[i] = -1e6;
#ifdef PF_DEBUG
    printf("%d: ", i);
#endif
    for (int j = 0; j < n_landmarks_; j++) {
      const Landmark &l = landmarks_[j];
      float dx = l.x - p.x,
            dy = l.y - p.y;
      float z = dx*C + dy*S,
            y = dx*S - dy*C;
      float diff = atan2f(y, z) - lm_bearing;
      float L = -LM_SELECTIVITY*diff*diff;
#ifdef PF_DEBUG
      printf("[%d]%f %f ", j, diff, L);
#endif
      if (L > LL[i]) {
        LL[i] = L;
      }
    }
#ifdef PF_DEBUG
    printf("LL[i]=%f\n", LL[i]);
#endif
    if (LL[i] > LLmax) {
      LLmax = LL[i];
    }
  }
#ifdef PF_DEBUG
  printf("LLmax=%f (%d landmarks)\n", LLmax, n_landmarks_);
#endif

  // now, normalize the distribution and resample particles
  float totalP = 0;
  for (int i = 0; i < n_particles_; i++) {
    LL[i] = exp(LL[i] - LLmax);
    totalP += LL[i];
#ifdef PF_DEBUG
    printf("%0.3f ", LL[i]);
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
    while (randP > LL[j]) {
      randP -= LL[j];
      j++;
      if (j == n_particles_) {
        j = 0;
      }
    }
    newp[i] = particles_[j];
#ifdef PF_DEBUG
    printf("%d ", j);
#endif
    randP += deltaP;
  }
#ifdef PF_DEBUG
  printf("\n");
#endif

  delete[] particles_;
  particles_ = newp;

  delete[] LL;
}

bool Localizer::GetLocationEstimate(Particle *mean) {
  mean->x = 0;
  mean->y = 0;
  mean->theta = 0;
  for (int i = 0; i < n_particles_; i++) {
    mean->x += particles_[i].x;
    mean->y += particles_[i].y;
    mean->theta += particles_[i].theta;
  }
  mean->x /= n_particles_;
  mean->y /= n_particles_;
  mean->theta /= n_particles_;
  return true;
}

}  // namespace coneslam
