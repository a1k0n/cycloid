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

  int ClosestIdx(float x, float y);

  // Lookup next local coordinate frame from global coordinates xg, yg, theta, index *i
  // Outputs updated *i (if changed), local x, y, cosine, sine
  void LocalState(float xg, float yg, float theta, int *i, float *xl, float *yl, float *cl, float *sl);

  bool GetTarget(float x, float y, int lookahead,
      float *closestx, float *closesty,
      float *normx, float *normy,
      float *kappa, float *lookahead_kappa);

 private:
  int n_pts_;
  TrajectoryPoint *pts_;
};

#endif  // DRIVE_TRAJTRACK_H_
