#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"

volatile bool done = false;

void handle_sigint(int signo) { done = true; }

int main() {
  I2C i2c;
  if (!i2c.Open()) {
    return 1;
  }

  signal(SIGINT, handle_sigint);

  IMU imu(i2c);
  printf("# t gx gy gz mx my mz ax ay az\n");

  while (!done) {
    timeval tv0;
    gettimeofday(&tv0, NULL);
#if 0
    IMUState s;
    if (imu.ReadCalibrated(&s)) {
      printf("%d.%06d m [%f,%f,%f] a [%f,%f,%f] g [%f,%f,%f]\n",
          tv0.tv_sec, tv0.tv_usec,
          s.N[0], s.N[1], s.N[2],
          s.g[0], s.g[1], s.g[2],
          s.w[0], s.w[1], s.w[2]);

      fflush(stdout);
    }
#else
    Eigen::Vector3f accel, gyro;
    float temp;
    if (imu.ReadIMU(&accel, &gyro, &temp)) {
      printf("%+0.4f %+0.4f %+0.4f | %+0.4f %+0.4f %+0.4f\n",
          gyro[0], gyro[1], gyro[2],
          accel[0], accel[1], accel[2]);
    }
#endif
    timeval tv;
    gettimeofday(&tv, NULL);
    int delay = 20000 - (tv.tv_usec % 20000);
    if (delay > 0)
      usleep(delay);
  }

  return 0;
}
