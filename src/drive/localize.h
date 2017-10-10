#ifndef DRIVE_LOCALIZE_H_
#define DRIVE_LOCALIZE_H_

#include <Eigen/Dense>

// on-line version of .././tools/replay/localize.py

class MapLocalizer {
 public:
  MapLocalizer();

  void ResetToStart();
  void ResetUnknown();

  // Use current state mean and covariance to predict next state
  void Predict(const Eigen::VectorXf &x,
     const Eigen::MatrixXf &P,
     float Delta_t);

  // Update distribution of s given curvature measurement / precision
  void Update(float k, float prec);

  // Get target trajectory given current location estimate
  float GetRacelineVelocity() const;
  float GetRacelineOffset() const;  // horizontal offset from center
  float GetRacelineAngle() const;  // angle w.r.t. center
  float GetRacelineCurvature() const;

  const Eigen::VectorXf& GetS() const { return p_s_; }

 private:
  // probability distribution of s coordinate along track
  Eigen::VectorXf p_s_;

  // for now, hardcode map data; make it loadable later
};

#endif  // DRIVE_LOCALIZE_H_
