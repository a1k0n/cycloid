#include <math.h>
#include <stdio.h>
#include "drive/trajtrack.h"

TrajectoryTracker::TrajectoryTracker() {
  n_pts_ = 0;
  pts_ = NULL;
}

TrajectoryTracker::~TrajectoryTracker() {
  delete[] pts_;
}

bool TrajectoryTracker::LoadTrack(const char *fname) {
  if (pts_ != NULL) {
    delete[] pts_;
  }

  FILE *fp = fopen(fname, "r");
  if (!fp) {
    perror(fname);
    return false;
  }

  if (fscanf(fp, "%d\n", &n_pts_) != 1) {
    fprintf(stderr, "failed loading %s\n", fname);
    fclose(fp);
    n_pts_ = 0;
    return false;
  }

  pts_ = new TrajectoryPoint[n_pts_];
  for (int i = 0; i < n_pts_; i++) {
    if (fscanf(fp, "%f %f %f %f %f\n",
        &pts_[i].x, &pts_[i].y,
        &pts_[i].nx, &pts_[i].ny,
        &pts_[i].k) != 5) {
      fprintf(stderr, "failed to load waypoint %d\n", i);
      fclose(fp);
      n_pts_ = 0;
      delete[] pts_;
      return false;
    }
  }

  fprintf(stderr, "*** loaded %d waypoints\n", n_pts_);

  fclose(fp);
  return true;
}

static float clip(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

bool TrajectoryTracker::GetTarget(float x, float y,
    float *closestx, float *closesty,
    float *normx, float *normy,
    float *kappa) {

  if (n_pts_ == 0) {
    return false;
  }

  int mini = 0;
  float mind = 1e12;

  for (int i = 0; i < n_pts_; i++) {
    const TrajectoryPoint &p = pts_[i];
    float dist = (p.x - x)*(p.x - x) + (p.y - y)*(p.y - y);
    if (dist < mind) {
      mind = dist;
      mini = i;
    }
  }

  const TrajectoryPoint &p = pts_[mini];
  *closestx = p.x;
  *closesty = p.y;
  *normx = p.nx;
  *normy = p.ny;
  *kappa = p.k;

  return true;
}
