// MPU-6050 (no magnetometer), MPU-9150, MPU-9250 supported

#include <unistd.h>
#include <Eigen/Dense>

#include "hw/gpio/i2c.h"
#include "hw/imu/mpu9150.h"

using Eigen::Vector3f;
using Eigen::VectorXd;
using Eigen::Matrix3f;
using Eigen::MatrixXd;

// acceleromter scale is 2g << ACCEL_SHIFT
// readout is (16384 >> ACCEL_SHIFT) LSB/g
const int ACCEL_SHIFT = 3;  // 0..3

MPU9150::~MPU9150() {}

bool MPU9150::Init() {
  i2c_.Write(0x68, 107, 0x80);  // reset
  usleep(10000);
  i2c_.Write(0x68, 107, 0);  // wake up
  i2c_.Write(0x68, 107, 1);  // use gyro clock
  i2c_.Write(0x68, 108, 0);  // enable accel + gyro
  i2c_.Write(0x68, 55, 0x32);  // enable bypass, int pin latch

  // samplerate divisor 4 -> 1kHz / 5 = 200Hz samplerate
  i2c_.Write(0x68, 25, 0x04);

  // dlpf_cfg = 3, no fsync; 41Hz gyro bandwidth, 5.9ms delay
  // 1kHz base sample rate
  i2c_.Write(0x68, 26, 0x03);

  // // dlpf_cfg = 5, no fsync; 10Hz gyro bandwidth, 13.8ms delay
  // 1kHz base sample rate
  // i2c_.Write(0x68, 26, 0x05);

  // fchoice: 11 (enables filter above)
  // gyro 1000deg/sec full scale
  // test  fs   - fchoice_b
  // 000 | 10 | 0 | 00
  i2c_.Write(0x68, 27, 0x10);

  // accel_fs_sel +/- 16g (11)
  i2c_.Write(0x68, 28, ACCEL_SHIFT << 3);

  // a_dlpfcfg = 6, low pass filter as much as possible
  i2c_.Write(0x68, 29, 0x06);

  i2c_.Write(0x68, 56, 1);  // DRDY int enable (for checking timing on scope)

  uint8_t id;
  i2c_.Read(0x68, 117, 1, &id);  // whoami
  fprintf(stderr, "\r\nMPU-9150 id: %02x\r\n", id);

#if 0  // let's not even use the magnetometer
  i2c_.Read(0x0c, 0x00, 1, &id);  // mag device id
  fprintf(stderr, "AK8975C id: %02x\r\n", id);

  uint8_t magadj8[3];
  i2c_.Write(0x0c, 0x0a, 0x0f);  // mag fuse rom access
  i2c_.Read(0x0c, 0x10, 3, magadj8);  // mag device id
  // i2c_.Write(0x0c, 0x0a, 0x01);  // single read (ak8975c only)
  i2c_.Write(0x0c, 0x0a, 0x16);  // 100Hz continuous 16-bit (ak8963c)
  magadj_ = Vector3f(
      1 + (magadj8[0] - 128.0f) / 256.0f,
      1 + (magadj8[1] - 128.0f) / 256.0f,
      1 + (magadj8[2] - 128.0f) / 256.0f);

  fprintf(stderr, "AK8975C mag adjust: %f %f %f\r\n",
         magadj_[0], magadj_[1], magadj_[2]);

#if 0
  mag_calibrated_ = LoadMagCalibration();
#endif
#endif

  return true;
}

bool MPU9150::ReadIMU(Vector3f *accel, Vector3f *gyro, float *temp) {
  uint8_t readbuf[14];
  // mpu-9150 accel & gyro
  if (i2c_.Read(0x68, 0x3b, 14, readbuf)) {
    int16_t ax = (readbuf[0] << 8) | readbuf[1],
            ay = (readbuf[2] << 8) | readbuf[3],
            az = (readbuf[4] << 8) | readbuf[5];
    int16_t t  = (readbuf[6] << 8) | readbuf[7];
    int16_t gx = (readbuf[ 8] << 8) | readbuf[ 9],
            gy = (readbuf[10] << 8) | readbuf[11],
            gz = (readbuf[12] << 8) | readbuf[13];
    // we are in 2048 LSB/g scale (+/- 16g)
    *accel = Vector3f(ax, ay, az) / (16384 >> ACCEL_SHIFT);
    *gyro = Vector3f(gx, gy, gz) * 1000.0 * M_PI / (180 * 32768.0);

    // the datasheet is completely useless for temperature
    *temp = t * (1.0/333.87) + 21;

    return true;
  }
  return false;
}
