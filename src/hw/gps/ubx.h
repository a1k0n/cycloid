#ifndef GPS_UBX_H_
#define GPS_UBX_H_

#include <stdint.h>

// Earth-Centered, Earth-Fixed position message from GPS
struct nav_posecef {
  uint32_t iTOW;  // time of week (ms)
  int32_t ecefX, ecefY, ecefZ;  // ECEF position (cm)
  uint32_t pAcc;  // position accuracy estimate (cm)
};

// 92 bytes
struct nav_pvt {
  uint32_t iTOW;  // ms time of week

  uint16_t year;
  uint8_t month;
  uint8_t day;

  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;

  uint32_t tAcc;  // ns time accuracy estimate
  int32_t nano;   // ns fraction of second, -1e9..1e9 (UTC)

  uint8_t fixType;  // 0: no fix, 1: dead reckoning, 2: 2D, 3: 3D,
                    // 4: GNSS+DR, 5: time-only
  uint8_t flags;
  uint8_t flags2;
  uint8_t numSV;

  int32_t lon;  // deg x 1e-7
  int32_t lat;  // deg x 1e-7
  int32_t height;  // mm above ellipsoid
  int32_t hMSL;    // mm above mean sea level
  uint32_t hAcc;  // mm horizontal accuracy
  uint32_t vAcc;  // mm vertical accuracy
  int32_t velN;  // mm/s NED north velocity
  int32_t velE;  // mm/s NED east velocity
  int32_t velD;  // mm/s NED down velocity
  int32_t gSpeed;  // mm/s ground speed (2D)
  int32_t headMot;  // 1e-5 deg heading of motion (2D)
  uint32_t sAcc;  // mm/s speed accuracy estimate
  uint32_t headAcc;  // 1e-5 deg heading accuracy estimate
  uint16_t pDOP;  // x0.01 position DOP
  uint8_t reserved1[6];

  int32_t headVeh;  // 1e-5 deg heading of vehicle (2D)
  int16_t magDec;  // 1e-2 deg magnetic declination
  uint16_t magAcc;  // 1e-2 deg magnetic declination accuracy
};

int ubx_open();
void ubx_read_loop(int fd, void (*on_pvt)(const nav_pvt&));

#endif  // GPS_UBX_H_
