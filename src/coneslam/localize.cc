#include <math.h>
#include <stdio.h>

#include "coneslam/localize.h"

namespace ConeSLAM {

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

}  // namespace ConeSLAM
