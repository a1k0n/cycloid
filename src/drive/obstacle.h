#ifndef DRIVE_OBSTACLE_H_
#define DRIVE_OBSTACLE_H_

#include <stdint.h>

class ObstacleDetector {
 public:
  ObstacleDetector();
  ~ObstacleDetector();

  bool Open(const char *lut_fname);

  void Update(const uint8_t *yuv420, uint8_t carthresh, uint8_t conethresh);

  const int32_t* GetConePenalties() const { return orange_sum_; }
  const int32_t* GetCarPenalties() const { return black_sum_; }

 private:
  int32_t black_sum_[256], orange_sum_[256];

  uint16_t *ymask_rle_;
  int ymask_rlelen_;
  int8_t *yanglemap_;
  uint16_t *uvmask_rle_;
  int uvmask_rlelen_;
  int8_t *uvanglemap_;
};

#endif  // DRIVE_OBSTACLE_H_
