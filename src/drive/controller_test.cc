// load up a track and see if the closed-loop control even works in theory

#include <Eigen/Dense>

#include "drive/config.h"
#include "drive/controller.h"
#include "drive/trajtrack.h"

const float M_K1 = 2.58;  // DC motor response constants (measured)
const float M_K2 = 0.093;
const float M_K3 = 0.218;
const float M_OFFSET = 0.103;  // minimum control input (dead zone)

// return new velocity after applying throttle
float motor(float u, float v, float dt) {
  if (u >= 0) {
    float dv = u*M_K1 - u*M_K2*v - M_K3*v;
    return v + dv*dt;
  } else {
    float dv = u*M_K2*v - M_K3*v;
    return v + dv*dt;
  }
}

int main() {
  DriverConfig config;
  DriveController control;

  float x = 8, y = -7, theta = 3.14;
  float v = 0, w = 0;
  float s = 0;
  uint16_t lastenc = 0;

  Eigen::Vector3f gyro(0, 0, 0), accel(0, 0, 0);

  const float dt = 1.0 / 30;

  for (int i = 0; i < 900; i++) {
    float throttle, steering;

    uint16_t encoders[4];
    uint16_t enc = 50*s;
    encoders[0] = encoders[1] = encoders[2] = encoders[3] = enc - lastenc;

    control.UpdateState(config, accel, gyro, 1, encoders, dt);
    control.UpdateLocation(config, x, y, theta);
    if (!control.GetControl(config, 0, 0, &throttle, &steering, dt, true, i)) {
      break;
    }

    printf("%d xy %f %f theta %f v %f w %f input %f %f control ", i, x, y, theta, v, w, throttle, steering);
    control.Dump();
    printf("\n");

    if (v > 0) {
      w = steering*v;
      gyro[2] = w;
    }
    v = motor(throttle, v, dt);
    theta += dt*w;
    float S = sin(theta), C = cos(theta);
    x += dt*v*C;
    y += dt*v*S;
    s += dt*v;
    lastenc = enc;
  }
}
