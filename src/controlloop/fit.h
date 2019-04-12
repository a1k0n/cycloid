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

class MotorSysId {
 public:
  MotorSysId() { Reset(); }

  void Reset(float scale=1) {
    lastv_ = 0;
    XTX.setIdentity();
    XTY << 100*scale, 0, 0, 0, -1;
  }

  void AddObservation(float v, float u, float dt) {
    Eigen::Matrix<float, 5, 1> X;
    float V = u > 0 ? 1 : 0;
    float dc = fabs(u);
    float dc2 = dc*dc;
    X << dc*V, dc2*V, dc*v, dc2*v, v;
    XTX *= 0.999;
    XTY *= 0.999;
    XTX += X * X.transpose();
    XTY += X * (v - lastv_) / dt;
    lastv_ = v;
  }

  Eigen::Matrix<float, 5, 1> Solve() {
    return XTX.ldlt().solve(XTY);
  }

  Eigen::Matrix<float, 5, 5> XTX;
  Eigen::Matrix<float, 5, 1> XTY;

 private:
  float lastv_;
};

#endif  // CONTROLLOOP_FIT_H_
