#ifndef DRIVE_TRAJTRACK_H_
#define DRIVE_TRAJTRACK_H_

struct TrajectoryPoint {
  float x, y;    // center of turn radius
  float nx, ny;  // "y" direction normal
  float k;       // curvature at this point
};

class TrajectoryTracker {
 public:
  TrajectoryTracker();
  ~TrajectoryTracker();

  bool LoadTrack(const char *fname);

  bool GetTarget(float x, float y, int lookahead,
      float *closestx, float *closesty,
      float *normx, float *normy,
      float *kappa, float *lookahead_kappa);

 private:
  int n_pts_;
  TrajectoryPoint *pts_;
};

#endif  // DRIVE_TRAJTRACK_H_
