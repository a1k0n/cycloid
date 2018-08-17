#include <stdio.h>
#include "drive/trajtrack.h"

bool TrajectoryTracker::LoadTrack(const char *fname) {
  FILE *fp = fopen(fname);
  if (!fp) {
    perror(fname);
    return false;
  }
  fclose(fp);
  return true;
}
