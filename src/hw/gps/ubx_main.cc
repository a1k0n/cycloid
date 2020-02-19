#include <stdio.h>

#include "./ubx.h"
#include "./vec3.h"

// show a local ECEF (earth-centered, earth-fixed) ground track in
// east/north/up coordinates

bool have_reference = false;
// reference position, taken from first measurement
vec3<uint32_t> ref_ecef(-1, -1, -1);
// orthonormal basis for local reference plane
vec3<double> ref_up, ref_north, ref_east;

// mean and precision (= 1/variance) of current north/east estimate
// eventually precision will be a 2x2 matrix once we have a heading and a
// predictive model, or just a bunch of particles

void compute_refplane() {
  static const double wgs84_inverse_flattening = 298.257223563;
  static const double wgs84_stretching = 1.0 / (wgs84_inverse_flattening - 1);
  ref_up = vec3<double>(ref_ecef.x, ref_ecef.y, ref_ecef.z);
  ref_up.z += ref_up.z * wgs84_stretching;
  ref_up.normalize();
  ref_north = (vec3<double>(0, 0, 1) - ref_up * ref_up.z).normalize();
  ref_east = cross(ref_north, ref_up);
}

/*
void OnECEF(const nav_posecef* navmsg) {
  fprintf(stderr, "POS-ECEF @%10d xyz (%d, %d, %d) +- %d\n", navmsg->iTOW,
          navmsg->ecefX, navmsg->ecefY, navmsg->ecefZ, navmsg->pAcc);
  if (!have_reference) {
    ref_ecef = vec3<uint32_t>(
        navmsg->ecefX, navmsg->ecefY, navmsg->ecefZ);
    have_reference = true;
    compute_refplane();
  } else {
    vec3<int32_t> relpos(
        navmsg->ecefX - ref_ecef.x,
        navmsg->ecefY - ref_ecef.y,
        navmsg->ecefZ - ref_ecef.z);
    vec3<double> local_pos(
        ref_east * relpos,
        ref_north * relpos,
        ref_up * relpos);
    printf("%0.0f %0.0f %0.0f\n",
           local_pos.x, local_pos.y, local_pos.z);
    fflush(stdout);
  }
}
*/

void OnPVT(const nav_pvt &msg) {
  printf("%04d-%02d-%02dT%02d:%02d:%02d.%09d ", msg.year, msg.month, msg.day,
      msg.hour, msg.min, msg.sec, msg.nano);
  printf("fix:%d numSV:%d %d.%07d +-%dmm %d.%07d +-%dmm height %dmm "
      "vel %d %d %d +-%d mm/s "
      "heading motion %d.%05d vehicle %d +- %d.%05d\n",
      msg.fixType, msg.numSV,
      msg.lon/10000000, abs(msg.lon)%10000000, msg.hAcc,
      msg.lat/10000000, abs(msg.lat)%10000000, msg.vAcc,
      msg.height,
      msg.velN, msg.velE, msg.velD, msg.sAcc,
      msg.headMot/100000, abs(msg.headMot)%100000,
      msg.headVeh, msg.headAcc/100000, msg.headAcc%100000);
}

int main(int argc, char** argv) {
  int ubxfd = ubx_open();
  if (ubxfd == -1) return 1;

  ubx_read_loop(ubxfd, OnPVT);

  return 0;
}
