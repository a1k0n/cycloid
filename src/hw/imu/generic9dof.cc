#include <byteswap.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <Eigen/Dense>
#include "imu/imu.h"

using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

namespace {
const uint8_t ADDR_ITG3200  = 0x68;
const uint8_t ADDR_HMC5883L = 0x1e;
const uint8_t ADDR_ADXL345  = 0x53;

const Vector3f gyrocal_b(-182.11, -179.32, -212.83);
const Vector3f gyrocal_m(-0.01169, -0.01123, -0.01164);
const float pointcloud_dist = 200.0f;

bool CheckPointCloud(const Vector3f& pt, std::vector<Vector3f> *pointcloud) {
  for (int i = 0; i < pointcloud->size(); i++) {
    if (((*pointcloud)[i] - pt).squaredNorm() <
        pointcloud_dist*pointcloud_dist) {
      return false;
    }
  }
  pointcloud->push_back(pt);
  return true;
}

}  // empty namespace

// our motion model should figure out whether the car is on the ground (1g down
// on the accelerometer) or flying off a ramp (0g); if it's on the ground and
// the motor is stopped (0 estimated velocity) then we can zero out the gyro,
// and calibrate the direction of the accelerometer Y direction.

bool IMU::Init() {
  mag_XTX_ = 0.01 * Matrix4f::Identity();
  mag_XTY_ = Vector4f::Zero();
  mag_bias_ = Vector3f::Zero();

  // config gyro
  i2c_.Write(ADDR_ITG3200, 0x3E, 0x01);  // use X gyro PLL oscillator
  i2c_.Write(ADDR_ITG3200, 0x15, 19);    // samplerate 50Hz (1000/(19+1))
  i2c_.Write(ADDR_ITG3200, 0x16, 0x18 + 4);  // enable, 20Hz bandwidth
  // config compass
  i2c_.Write(ADDR_HMC5883L, 0x00, 0x38);  // CRA: 75Hz rate w/ 2 averages
  i2c_.Write(ADDR_HMC5883L, 0x01, 0x20);  // CRB: set gain 1090 LSB/Gauss
  i2c_.Write(ADDR_HMC5883L, 0x02, 0x00);  // continuous measurement
  // config accelerometer
  i2c_.Write(ADDR_ADXL345, 0x2c, 0x09);  // 25Hz bw, 50Hz samplerate
  i2c_.Write(ADDR_ADXL345, 0x31, 0x08);  // FULL_RES
  i2c_.Write(ADDR_ADXL345, 0x38, 0x00);  // bypass FIFO, sample @ 50Hz
  i2c_.Write(ADDR_ADXL345, 0x2d, 0x08);  // turn on

  return true;
}

bool IMU::ReadRaw(IMURawState *s) {
  uint8_t axis_buf[8];
  if (!i2c_.Read(ADDR_ITG3200, 0x1b, axis_buf, 8))
    return false;
  // temperature is 280 LSB/deg C, -13200 LSB @35 C
  s->gyro_temp = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));
  s->gyro_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // roll
  s->gyro_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // pitch
  s->gyro_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+6));  // yaw

  if (!i2c_.Read(ADDR_HMC5883L, 0x03, axis_buf, 6))
    return false;
  s->mag_x = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+0));  // front?
  s->mag_z = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+2));  // up
  s->mag_y = bswap_16(*reinterpret_cast<uint16_t*>(axis_buf+4));  // side?

  if (!i2c_.Read(ADDR_ADXL345, 0x32, axis_buf, 6))
    return false;
  s->accel_x = (*reinterpret_cast<uint16_t*>(axis_buf+0));  // toward back
  s->accel_y = (*reinterpret_cast<uint16_t*>(axis_buf+2));  // toward right
  s->accel_z = (*reinterpret_cast<uint16_t*>(axis_buf+4));  // toward ground

  return true;
}

bool IMU::ReadCalibrated(IMUState *s) {
  IMURawState rawstate;
  if (!ReadRaw(&rawstate))
    return false;

  Calibrate(rawstate, s);
  return true;
}

// all devices are oriented such that X is to the front of the car, Y is to
// the left, and Z is up.
void IMU::Calibrate(const IMURawState &s, IMUState *state) {
  // gyro offset calibration (rough -- needs work)
  Vector3f w0(-s.gyro_y, -s.gyro_z, s.gyro_x);
  w0 -= gyrocal_b + s.gyro_temp * gyrocal_m;
  // scale to radians/second
  state->w = w0 * M_PI / (180.0 * 14.375);

  // magnetometer calibration via least-squares fit sphere
  // TODO: fix the axes (i think x and z are swapped and one is negated)
  Vector3f m(s.mag_x, s.mag_y, s.mag_z);
  // if there's room in the point cloud for this new point, update calibration
  if (CheckPointCloud(m, &mag_cal_points_)) {
    Vector4f b(s.mag_x, s.mag_y, s.mag_z, 1);
    mag_XTX_ += b * b.transpose();
    float bmag = b.squaredNorm();
    mag_XTY_ += bmag * b;
    Vector4f beta = mag_XTX_.ldlt().solve(mag_XTY_);
    mag_bias_ = beta.block<3, 1>(0, 0) / 2;
    bmag = sqrt(beta[3] + mag_bias_.squaredNorm());
    fprintf(stderr, "imu: updated mag calibration [%f %f %f] R=%f\n",
            beta[0] / 2, beta[1] / 2, beta[2] / 2, bmag);
  }
  state->N = m - mag_bias_;

  // TODO: verify axes
  state->g = Vector3f(-s.accel_y, -s.accel_z, -s.accel_x);
}
