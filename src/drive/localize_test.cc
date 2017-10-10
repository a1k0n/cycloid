#include "drive/localize.h"

#include <iostream>

int main(int argc, char **argv) {
  MapLocalizer l;

  l.ResetToStart();
  std::cout << l.GetS().transpose() << std::endl;

  Eigen::VectorXf x = Eigen::VectorXf::Zero(6);
  x << 2.5, 0, 0, 0, 0, 0;  // set velocity, leave others empty
  Eigen::MatrixXf P = Eigen::MatrixXf::Identity(6, 6);

  l.Predict(x, P, 0.2);
  std::cout << l.GetS().transpose() << std::endl;

  x[0] = 0.5;
  l.Predict(x, P, 0.2);
  std::cout << l.GetS().transpose() << std::endl;

#if 1
  // now spread things out a bunch
  x[0] = 1;
  for (int i = 0; i < 96*5; i++) {
    l.Predict(x, P, 0.2);
  }
  std::cout << l.GetS().transpose() << std::endl;

#endif

  // and make a measurement
  l.Update(-0.3, 10);
  std::cout << l.GetS().transpose() << std::endl;

  std::cout << "v: " << l.GetRacelineVelocity() << std::endl;
  std::cout << "k: " << l.GetRacelineCurvature() << std::endl;
  std::cout << "y: " << l.GetRacelineOffset() << std::endl;
  std::cout << "psi: " << l.GetRacelineAngle() << std::endl;
}
