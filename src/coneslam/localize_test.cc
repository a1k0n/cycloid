#include <stdio.h>
#include "coneslam/localize.h"
#include "drive/config.h"

using coneslam::Localizer;
using coneslam::Particle;

const char *testdata_file = "../src/coneslam/testdata/194625.txt";

int main() {
  Localizer loc(300);
  if (!loc.LoadLandmarks("../src/coneslam/testdata/lm.txt")) {
    return 1;
  }

  FILE *fp = fopen(testdata_file, "r");
  if (!fp) {
    perror(testdata_file);
    return 1;
  }

  Particle p;
  loc.GetLocationEstimate(&p);
  printf("initial location %f %f %f\n", p.x, p.y, p.theta);

  DriverConfig config;
  config.Load();

  float dt, ds, w;
  int nLM;
  int frame = 0;
  while (fscanf(fp, "%f %f %f %d\n", &dt, &ds, &w, &nLM) == 4) {
    loc.Predict(ds, w, dt);
    for (int j = 0; j < nLM; j++) {
      float lm_bearing;
      fscanf(fp, "%f\n", &lm_bearing);
      loc.UpdateLM(lm_bearing, config.lm_precision,
                   config.lm_bogon_thresh*0.01);
    }
    loc.GetLocationEstimate(&p);
    printf("%d: %f %f %f\n", frame++, p.x, p.y, p.theta);
  }
}
