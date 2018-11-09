#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"

using Eigen::Vector3f;

int main() {
  I2C i2c;
  IMU imu(i2c);

  if (!i2c.Open()) {
    return 1;
  }

  if (!imu.Init()) {
    return 1;
  }

  for (;;) {
    Vector3f acc, gyro;
    float temp;
    if (imu.ReadIMU(&acc, &gyro, &temp)) {
      printf("a [%f,%f,%f] g [%f,%f,%f] temp %0.2f C\n",
          acc[0], acc[1], acc[2],
          gyro[0], gyro[1], gyro[2], temp);
      fflush(stdout);
    }
  }

  return 0;
}
