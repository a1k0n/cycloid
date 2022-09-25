#include <stdio.h>

#include "drive/pinet.h"
#include "drive/trajtrack.h"

const char *testdata_file = "../src/drive/testdata/track2.txt";

int main() {
  TrajectoryTracker tt;
  PiNetwork pi;

  if (!tt.LoadTrack(testdata_file)) {
    return 1;
  }

  if (!pi.Load("../src/drive/testdata/pinet.bin")) {
    return 1;
  }

  float xg = 11.494, yg = -3.9093, theta = 6.2637;

  int i = tt.ClosestIdx(xg, yg);
  printf("ClosestIdx(%f, %f) = %d\n", xg, yg, i);

  xg = 11.6269, yg = -3.9280, theta = 6.1432;

  float xl, yl, cl, sl;
  tt.LocalState(xg, yg, theta, &i, &xl, &yl, &cl, &sl);
  printf("LocalState(%f, %f, %f) = %d, %f, %f, %f, %f\n", xg, yg, theta, i, xl, yl, cl, sl);
  
  xg = 11.7509, yg = -3.9617, theta = 6.0173;
  tt.LocalState(xg, yg, theta, &i, &xl, &yl, &cl, &sl);
  printf("LocalState(%f, %f, %f) = %d, %f, %f, %f, %f\n", xg, yg, theta, i, xl, yl, cl, sl);

  float u1, u2;
  pi.Action(i, 5, -1, xl, yl, cl, sl, 1, 1, &u1, &u2);
  printf("%f %f", u1, u2);
}
