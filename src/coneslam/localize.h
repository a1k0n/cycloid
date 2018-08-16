#ifndef CONESLAM_LOCALIZE_H_
#define CONESLAM_LOCALIZE_H_

namespace ConeSLAM {

#include <stdlib.h>

struct Particle {
  float x, y, theta;
};

// for now we will assume all landmarks have the same (round) covariance
struct Landmark {
  float x, y;
};

// Localization, assuming cone locations are all known
class Localizer {
 public:
  explicit Localizer(int n_particles) {
    n_particles_ = n_particles;
    particles_ = new Particle[n_particles];
    Reset();
  }

  bool LoadLandmarks(const char *filename);

  void Reset();

  void Predict(float ds, float w, float dt);
  void UpdateLM(float lm_bearing);

  bool GetLocationEstimate(Particle *mean);

 private:
  int n_particles_;
  Particle *particles_;

  int n_landmarks_;
  Landmark *landmarks_;
};

}  // namespace ConeSLAM

#endif  // CONESLAM_LOCALIZE_H_
