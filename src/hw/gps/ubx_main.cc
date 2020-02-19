#include <stdio.h>
#include <algorithm>

#include "hw/gps/ubx.h"

class NavLogger: public NavListener {
 public:
  void OnNav(const nav_pvt& msg) {
    printf("%04d-%02d-%02dT%02d:%02d:%02d.%09d ", msg.year, msg.month, msg.day,
           msg.hour, msg.min, msg.sec, msg.nano);
    printf(
        "fix:%d numSV:%d %d.%07d +-%dmm %d.%07d +-%dmm height %dmm "
        "vel %d %d %d +-%d mm/s "
        "heading motion %d.%05d vehicle %d +- %d.%05d\n",
        msg.fixType, msg.numSV, msg.lon / 10000000,
        std::abs(msg.lon) % 10000000, msg.hAcc, msg.lat / 10000000,
        std::abs(msg.lat) % 10000000, msg.vAcc, msg.height, msg.velN, msg.velE,
        msg.velD, msg.sAcc, msg.headMot / 100000,
        std::abs(msg.headMot) % 100000, msg.headVeh, msg.headAcc / 100000,
        msg.headAcc % 100000);
  }
} logger;

int main(int argc, char** argv) {
  int ubxfd = ubx_open();
  if (ubxfd == -1) return 1;

  ubx_read_loop(ubxfd, &logger);

  return 0;
}
