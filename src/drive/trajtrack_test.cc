#include <stdio.h>

#include "drive/trajtrack.h"

const char *testdata_file = "../src/drive/testdata/track.txt";

int main() {
  TrajectoryTracker tt;

  if (!tt.LoadTrack(testdata_file)) {
    return 1;
  }

  float cx, cy, nx, ny, k, t;
  if (!tt.GetTarget(0, 0, &cx, &cy, &nx, &ny, &k, &t)) {
    fprintf(stderr, "failed to get target?!");
    return 1;
  }

  printf("%f %f %f %f %f\n", cx, cy, nx, ny, k);

  if (!tt.GetTarget(51, 20, &cx, &cy, &nx, &ny, &k, &t)) {
    fprintf(stderr, "failed to get target?!");
    return 1;
  }

  printf("%f %f %f %f %f\n", cx, cy, nx, ny, k);
}
