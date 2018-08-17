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
    if (fscanf(fp, "%f %f %f %f %f %f %f %f %f\n",
        &pts_[i].x, &pts_[i].y, &pts_[i].r,
        &pts_[i].lx0, &pts_[i].ly0,
        &pts_[i].lx1, &pts_[i].ly1,
        &pts_[i].nx, &pts_[i].ny) != 9) {
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
    float *kappa,
    float *lineposition) {

  if (n_pts_ == 0) {
    return false;
  }

  int mini = 0;
  float mind = 1e12;
  float minx, miny, mint;

  for (int i = 0; i < n_pts_; i++) {
    const TrajectoryPoint &p = pts_[i];
    // project x, y onto (p.lx0, p.ly0)..(p.lx1, p.ly1)
    float dpx = p.lx1 - p.lx0;
    float dpy = p.ly1 - p.ly0;
    float tnum = dpx*(x - p.lx0) + dpy*(y - p.ly0);
    float tden = dpx*dpx + dpy*dpy;

    float t = clip(tnum / tden, 0, 1);
    float px = p.lx0*(1-t) + p.lx1*t;
    float py = p.ly0*(1-t) + p.ly1*t;
    float dist = (px - x)*(px - x) + (py - y)*(py - y);
    if (dist < mind) {
      mind = dist;
      mint = t;
      mini = i;
      minx = px;
      miny = py;
    }
  }

  if (mint == 1) {  // on next circle; just advance i
    mint = 0;
    mini = (mini+1) % n_pts_;
  }

  const TrajectoryPoint &p = pts_[mini];
  if (mint == 0) {  // on circle
    // recompute closest x, y from circle
    float dpx = x - p.x;
    float dpy = y - p.y;
    float norm = sqrt(dpx*dpx + dpy*dpy);
    *closestx = p.x + fabs(p.r) * dpx / norm;
    *closesty = p.y + fabs(p.r) * dpy / norm;
    *normx = -dpx * copysignf(1.0, p.r) / norm;
    *normy = -dpy * copysignf(1.0, p.r) / norm;
    *kappa = 1.0 / p.r;
    *lineposition = 0;
  } else {  // on line segment
    *closestx = minx;
    *closesty = miny;
    *normx = p.nx;
    *normy = p.ny;
    *kappa = 0;
    *lineposition = mint;
  }
  return true;
}
