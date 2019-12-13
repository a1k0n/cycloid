// ICM-20600 accel + gyro, no magnetometer

#include <unistd.h>
#include <Eigen/Dense>

#include "hw/gpio/i2c.h"
#include "hw/imu/invensense.h"

using Eigen::Vector3f;
using Eigen::VectorXd;
using Eigen::Matrix3f;
using Eigen::MatrixXd;

// acceleromter scale is 2g << ACCEL_SHIFT
// readout is (16384 >> ACCEL_SHIFT) LSB/g
const int ACCEL_SHIFT = 3;  // 0..3

InvensenseIMU::~InvensenseIMU() {}

bool InvensenseIMU::Init() {
  if (!i2c_.Write(i2caddr_, 107, 0x80)) {  // reset
    return false;
  }
  usleep(10000);
  i2c_.Write(i2caddr_, 107, 0);  // wake up
  i2c_.Write(i2caddr_, 107, 1);  // use PLL clock
  i2c_.Write(i2caddr_, 108, 0);  // enable accel + gyro

  // samplerate divisor 4 -> 1kHz / 5 = 200Hz samplerate
  // samplerate divisor 9 -> 1kHz / 10 = 100Hz samplerate
  i2c_.Write(i2caddr_, 25, 0x04);

  // dlpf_cfg = 3, no ext_sync; 41Hz gyro bandwidth, 5.9ms delay
  // 1kHz base sample rate
  i2c_.Write(i2caddr_, 26, 0x03);

  // // dlpf_cfg = 5, no fsync; 10Hz gyro bandwidth, 13.8ms delay
  // 1kHz base sample rate
  // i2c_.Write(i2caddr_, 26, 0x05);

  // fchoice: 11 (enables filter above)
  // gyro 1000deg/sec full scale
  // test  fs   - fchoice_b
  // 000 | 10 | 0 | 00
  i2c_.Write(i2caddr_, 27, 0x10);

  // accel_fs_sel +/- 16g (11)
  i2c_.Write(i2caddr_, 28, ACCEL_SHIFT << 3);

  // a_dlpfcfg = 6, low-pass filter as much as possible (5Hz!)
  // i2c_.Write(i2caddr_, 29, 0x06);
  // no way, that sucks

  // 44.8Hz accel bandwidth, which should be good for our 100Hz samplerate
  i2c_.Write(i2caddr_, 29, 0x03);

  uint8_t id;
  if (!i2c_.Read(i2caddr_, 117, 1, &id)) {  // whoami
    return false;
  }
  fprintf(stderr, "\r\ninvensense IMU id: %02x\r\n", id);

  return true;
}

bool InvensenseIMU::ReadIMU(Vector3f *accel, Vector3f *gyro) {
  uint8_t readbuf[14];
  // mpu-9150 accel & gyro
  if (i2c_.Read(i2caddr_, 0x3b, 14, readbuf)) {
    int16_t ax = (readbuf[0] << 8) | readbuf[1],
            ay = (readbuf[2] << 8) | readbuf[3],
            az = (readbuf[4] << 8) | readbuf[5];
    // int16_t t  = (readbuf[6] << 8) | readbuf[7];
    int16_t gx = (readbuf[ 8] << 8) | readbuf[ 9],
            gy = (readbuf[10] << 8) | readbuf[11],
            gz = (readbuf[12] << 8) | readbuf[13];
    // we are in 2048 LSB/g scale (+/- 16g)
    *accel = Vector3f(ax, ay, az) * zorient_ / (16384 >> ACCEL_SHIFT);

    // we are in +/- 1000 degrees/second full scale range
    // return radians/second
    *gyro = Vector3f(gx, gy, gz) * 1000.0 * M_PI * zorient_ / (180 * 32768.0);

    // the datasheet is completely useless for temperature
    // *temp = t * (1.0/333.87) + 21;

    return true;
  }
  return false;
}

