#ifndef GPSDRIVE_TRAJTRACK_H_
#define GPSDRIVE_TRAJTRACK_H_

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

#endif  // GPSDRIVE_TRAJTRACK_H_
