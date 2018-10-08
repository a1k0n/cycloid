// "tape" track -- circles with lines connecting them,
// like on a tape reel machine
#ifndef DRIVE_TAPETRACK_H_
#define DRIVE_TAPETRACK_H_

struct TapeTrackPoint {
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
  ~TrajectoryTracker();

  bool LoadTrack(const char *fname);

  // lineposition is 0..1; can be used to slow down before the next turn
  bool GetTarget(float x, float y,
      float *closestx, float *closesty,
      float *normx, float *normy,
      float *kappa,
      float *lineposition);

 private:
  int n_pts_;
  TapeTrackPoint *pts_;
};

#endif  // DRIVE_TAPETRACK_H_
