#ifndef DRIVE_TRAJTRACK_H_
#define DRIVE_TRAJTRACK_H_

struct TrajectoryPoint {
  float x, y;  // center of turn radius
  float r;  // turn radius (positive: CW; negative: CCW)
  // line between this cone and next:
  float lx0, ly0;  // starting point of line on this turn
  float lx1, ly1;  // ending point on next turn radius
  float nx, ny;  // track normal along line
};

class TrajectoryTracker {
 public:
  TrajectoryTracker();
  bool LoadTrack(const char *fname);

  // lineposition is 0..1; can be used to slow down before the next turn
  bool GetTarget(float x, float y,
      float *closestx, float *closesty,
      float *normx, float *normy,
      float *kappa,
      float *lineposition);

 private:
  int n_pts_;
  TrajectoryPoint *pts_;
};

#endif  // DRIVE_TRAJTRACK_H_
