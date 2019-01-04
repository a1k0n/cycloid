// ICM-20600 accel + gyro, no magnetometer

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>

#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"

using Eigen::Vector3f;
using Eigen::VectorXd;
using Eigen::Matrix3f;
using Eigen::MatrixXd;

// acceleromter scale is 2g << ACCEL_SHIFT
// readout is (16384 >> ACCEL_SHIFT) LSB/g
const int ACCEL_SHIFT = 3;  // 0..3

bool IMU::Init() {
  i2c_.Write(0x69, 107, 0x80);  // reset
  usleep(10000);
  i2c_.Write(0x69, 107, 0);  // wake up
  i2c_.Write(0x69, 107, 1);  // use PLL clock
  i2c_.Write(0x69, 108, 0);  // enable accel + gyro

  // samplerate divisor 4 -> 1kHz / 5 = 200Hz samplerate
  // samplerate divisor 9 -> 1kHz / 10 = 100Hz samplerate
  i2c_.Write(0x69, 25, 0x04);

  // dlpf_cfg = 3, no ext_sync; 41Hz gyro bandwidth, 5.9ms delay
  // 1kHz base sample rate
  i2c_.Write(0x69, 26, 0x03);

  // // dlpf_cfg = 5, no fsync; 10Hz gyro bandwidth, 13.8ms delay
  // 1kHz base sample rate
  // i2c_.Write(0x69, 26, 0x05);

  // fchoice: 11 (enables filter above)
  // gyro 1000deg/sec full scale
  // test  fs   - fchoice_b
  // 000 | 10 | 0 | 00
  i2c_.Write(0x69, 27, 0x10);

  // accel_fs_sel +/- 16g (11)
  i2c_.Write(0x69, 28, ACCEL_SHIFT << 3);

  // a_dlpfcfg = 6, low-pass filter as much as possible
  i2c_.Write(0x69, 29, 0x06);

  uint8_t id;
  i2c_.Read(0x69, 117, 1, &id);  // whoami
  fprintf(stderr, "\r\nICM-20600 id: %02x\r\n", id);

  return true;
}

bool IMU::ReadIMU(Vector3f *accel, Vector3f *gyro, float *temp) {
  uint8_t readbuf[14];
  // mpu-9150 accel & gyro
  if (i2c_.Read(0x69, 0x3b, 14, readbuf)) {
    int16_t ax = (readbuf[0] << 8) | readbuf[1],
            ay = (readbuf[2] << 8) | readbuf[3],
            az = (readbuf[4] << 8) | readbuf[5];
    int16_t t  = (readbuf[6] << 8) | readbuf[7];
    int16_t gx = (readbuf[ 8] << 8) | readbuf[ 9],
            gy = (readbuf[10] << 8) | readbuf[11],
            gz = (readbuf[12] << 8) | readbuf[13];
    // we are in 2048 LSB/g scale (+/- 16g)
    *accel = Vector3f(ax, ay, az) / (16384 >> ACCEL_SHIFT);

    // we are in +/- 1000 degrees/second full scale range
    // return radians/second
    *gyro = Vector3f(gx, gy, gz) * 1000.0 * M_PI / (180 * 32768.0);

    // the datasheet is completely useless for temperature
    *temp = t * (1.0/333.87) + 21;

    return true;
  }
  return false;
}

