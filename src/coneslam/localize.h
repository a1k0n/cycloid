#ifndef CONESLAM_LOCALIZE_H_
#define CONESLAM_LOCALIZE_H_

#include <stdlib.h>

namespace coneslam {

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
    n_landmarks_ = 0;
    landmarks_ = NULL;
    Reset();
  }

  ~Localizer();

  bool LoadLandmarks(const char *filename);

  void Reset();

  // predict after encoder / gyro measurement
  void Predict(float ds, float w, float dt);
  // update after landmark measurement
  void UpdateLM(float lm_bearing, float precision);

  bool GetLocationEstimate(Particle *mean);

  const Landmark *GetLandmarks() const { return landmarks_; }
  int NumLandmarks() const { return n_landmarks_; }

  const Particle *GetParticles() const { return particles_; }
  int NumParticles() const { return n_particles_; }

 private:
  int n_particles_;
  Particle *particles_;

  int n_landmarks_;
  Landmark *landmarks_;
};

}  // namespace coneslam

#endif  // CONESLAM_LOCALIZE_H_
