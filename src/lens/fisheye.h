#ifndef LENS_FISHEYE_H_
#define LENS_FISHEYE_H_

class FisheyeLens {
 public:
  void SetCalibration(float fx, float fy, float cx, float cy, float k1) {
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    this->k1 = k1;
  }

  bool LoadCalibration(const char *fname);

  // generate a wxhx3 float undistorted map of every point on the image.
  // z is either -1 or 1 depending on whether the ray is ahead of or behind the
  // image plane
  float *GenUndistortedPts(int w, int h) const;

  void DistortPoint(float x, float y, float z, float *u, float *v) const;

 private:
  float fx, fy;  // angular focal length (x, y)
  float cx, cy;  // optical center
  float k1;      // distortion coefficient
};

#endif  // LENS_FISHEYE_H_