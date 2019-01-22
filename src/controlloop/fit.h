#ifndef CONTROLLOOP_FIT_H_
#define CONTROLLOOP_FIT_H_

// least-squares system identification for 2nd-order systems

#include <Eigen/Dense>

class SysIdentifier {
 public:
  SysIdentifier() { Reset(); }

  void Reset() {
    XTY = Eigen::Vector4f::Zero();
    XTX = 1e-3 * Eigen::Matrix4f::Identity();
    last_x = 0;
    last_y = 0;
    last_dxdt = 0;
  }

  void AddObservation(float x, float scale, float y, float dt) {
    float dxdt = (x - last_x) / dt;
    float d2xdt2 = (dxdt - last_dxdt) / dt;

    Eigen::Vector4f X(x, dxdt, d2xdt2, scale);
    XTY += X * last_y * scale;
    XTX += X * X.transpose();

    last_x = x;
    last_y = y;
    last_dxdt = dxdt;
  }

  Eigen::Vector4f Solve() {
    Eigen::LDLT<Eigen::Matrix4f> ALDLT = XTX.ldlt();
    return ALDLT.solve(XTY);
  }

  Eigen::Matrix4f XTX;
  Eigen::Vector4f XTY;

 private:
  float last_x;
  float last_dxdt;
  float last_y;
};

#endif  // CONTROLLOOP_FIT_H_
