#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"

using Eigen::Vector3f;

void sighandler(int sig) {}

const int INTERVAL_us = 2500;

int main() {
  I2C i2c;
  IMU imu(i2c);

  if (!i2c.Open()) {
    return 1;
  }

  if (!imu.Init()) {
    return 1;
  }

  signal(SIGALRM, sighandler);

  // round up next expiration to the nearest 5ms (sampling @ 200Hz)
  timeval tv;
  gettimeofday(&tv, NULL);
  itimerval timerval = {
    {0, INTERVAL_us},
    {0, INTERVAL_us - (tv.tv_usec % INTERVAL_us)}
  };
  if (setitimer(ITIMER_REAL, &timerval, NULL) != 0) {
    perror("setitimer");
  }

  for (;;) {
    Vector3f acc, gyro;
    float temp;
    gettimeofday(&tv, NULL);
    if (imu.ReadIMU(&acc, &gyro, &temp)) {
      printf("%ld.%06ld a [%f,%f,%f] g [%f,%f,%f] temp %0.2f C\n",
          tv.tv_sec, tv.tv_usec,
          acc[0], acc[1], acc[2],
          gyro[0], gyro[1], gyro[2], temp);
      fflush(stdout);
    }
    pause();
  }

  return 0;
}
