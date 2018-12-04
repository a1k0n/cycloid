#include <stdio.h>

#include "drive/trajtrack.h"

const char *testdata_file = "../src/drive/testdata/track.txt";

int main() {
  TrajectoryTracker tt;

  if (!tt.LoadTrack(testdata_file)) {
    return 1;
  }

  float cx, cy, nx, ny, k, t;
  if (!tt.GetTarget(/*x=*/ 0, /*y=*/0, /*lookahead=*/12, /*closestx=*/&cx,
          /*closesty=*/&cy, /*normx=*/&nx, /*normy=*/&ny, /*kappa=*/&k,
          /*lookahead_kappa=*/&t)) {
    fprintf(stderr, "failed to get target?!");
    return 1;
  }

  printf("%f %f %f %f %f\n", cx, cy, nx, ny, k);

  if (!tt.GetTarget(/*x=*/ 51, /*y=*/20, /*lookahead=*/42, /*closestx=*/&cx,
            /*closesty=*/&cy, /*normx=*/&nx, /*normy=*/&ny, /*kappa=*/&k,
            /*lookahead_kappa=*/&t)) {
    fprintf(stderr, "failed to get target?!");
    return 1;
  }

  printf("%f %f %f %f %f\n", cx, cy, nx, ny, k);
}
