#ifndef CONESLAM_LOCALIZE_H_
#define CONESLAM_LOCALIZE_H_

#include <stdlib.h>
#include <stdint.h>

namespace coneslam {

struct Particle {
  float x, y, theta, heading;
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
    LL_ = new float[n_particles];
    home_x_ = home_y_ = home_theta_ = 0;
    Reset();
  }

  ~Localizer();

  bool LoadLandmarks(const char *filename);

  void Reset();

  // predict after encoder / gyro measurement
  // ds is in meters, w in rad/sec, dt in sec
  void Predict(float ds, float w, float dt);

  // update after landmark measurement
  void UpdateLM(float lm_bearing, float precision, float bogon_thresh);
  void Resample();  // implicitly resets internal likelihoods

  bool GetLocationEstimate(Particle *mean) const;

  const Landmark *GetLandmarks() const { return landmarks_; }
  int NumLandmarks() const { return n_landmarks_; }

  const Particle *GetParticles() const { return particles_; }
  int NumParticles() const { return n_particles_; }

  int SerializedSize() const;
  int Serialize(uint8_t *buf, int buflen) const;

 private:
  void ResetLikelihood();

  int n_particles_;
  Particle *particles_;

  int n_landmarks_;
  Landmark *landmarks_;

  float home_x_, home_y_, home_theta_;

  float *LL_;  // particle log-likelihood
  float LLmax_;
};

}  // namespace coneslam

#endif  // CONESLAM_LOCALIZE_H_
