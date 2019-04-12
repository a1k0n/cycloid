#include "controlloop/motorcontrol.h"
#include <math.h>

int main() {
  SelfTuningMotorControl mc;

  const float J = 1;
  const float k1 = 33.7 * J, k2 = 1.44 * J, k3 = -0.0516 * J,
              k4 = -0.000676 * J, k5 = -2.14 * J;

  const char *fname = TESTDATA_PATH "/testlog.txt";
  FILE *fp = fopen(fname, "r");
  if (!fp) {
    perror(fname);
    return 1;
  }

  // ignore header
  char buf[1024];
  fgets(buf, sizeof(buf), fp);

  float dt, controlx, controly, u_esc, u_steer, dw, wperiod;
  float gx, gy, gz;
  float ax, ay, az;
  float sim_v = 0;
  while (fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f", &dt, &controlx,
                &controly, &u_esc, &u_steer, &dw, &wperiod, &gx, &gy, &gz, &ax,
                &ay, &az) == 13) {
    float v = dw / dt;
    if (wperiod != 0) {
      v = 1e6 / wperiod;
    }

    float u = static_cast<float>(mc.Control(v, sim_v, 6.28 * 5, dt));
    float V = u > 0 ? 1 : 0;
    float dc = fabsf(u);
    sim_v += dt * (k1 * V * dc + k2 * V * dc * dc + k3 * sim_v * dc +
                   k4 * sim_v * dc * dc + k5 * sim_v);
    // printf("u %f %f/%f\n", u, v, sim_v);
  }
  fclose(fp);

  printf("identified parameters in simulation:\n");
  auto k = mc.sysid_.Solve();
  printf("%f %f %f %f %f\n", k[0], k[1], k[2], k[3], k[4]);
  printf("true parameters:");
  printf("%f %f %f %f %f\n", k1, k2, k3, k4, k5);

  return 0;
}
