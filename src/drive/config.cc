// AUTO-GENERATED! Edit conf.py instead.

#include <stdio.h>
#include <string.h>
#include "drive/config.h"

const char *DriverConfig::confignames[] = {
  "speed limit",
  "traction limit",
  "lookahead dist",
  "lookahead time",
  "lookahead kmax",
  "path penalty",
  "cone penalty",
  "car penalty",
  "motor gain",
  "motor kI",
  "servo rate",
  "servo offset",
  "servo kI",
  "servo min",
  "servo max",
};

const int DriverConfig::N_CONFIGITEMS = 15;

bool DriverConfig::Save() {
  FILE *fp = fopen("driverconf.txt", "w");
  if (!fp) {
    perror("driverconf.txt");
    return false;
  }
  fprintf(fp, "speed_limit          %d\n", speed_limit);
  fprintf(fp, "traction_limit       %d\n", traction_limit);
  fprintf(fp, "lookahead_dist       %d\n", lookahead_dist);
  fprintf(fp, "lookahead_time       %d\n", lookahead_time);
  fprintf(fp, "lookahead_kmax       %d\n", lookahead_kmax);
  fprintf(fp, "path_penalty         %d\n", path_penalty);
  fprintf(fp, "cone_penalty         %d\n", cone_penalty);
  fprintf(fp, "car_penalty          %d\n", car_penalty);
  fprintf(fp, "motor_gain           %d\n", motor_gain);
  fprintf(fp, "motor_kI             %d\n", motor_kI);
  fprintf(fp, "servo_rate           %d\n", servo_rate);
  fprintf(fp, "servo_offset         %d\n", servo_offset);
  fprintf(fp, "servo_kI             %d\n", servo_kI);
  fprintf(fp, "servo_min            %d\n", servo_min);
  fprintf(fp, "servo_max            %d\n", servo_max);

  fclose(fp);
  return true;
}

bool DriverConfig::Load() {
  FILE *fp = fopen("driverconf.txt", "r");
  if (!fp) {
    perror("driverconf.txt");
    return false;
  }
  char varbuf[21];
  int valuebuf;
  while (fscanf(fp, "%20s %d", varbuf, &valuebuf) == 2) {
         if (!strcmp(varbuf, "speed_limit"))       { speed_limit          = valuebuf; }
    else if (!strcmp(varbuf, "traction_limit"))    { traction_limit       = valuebuf; }
    else if (!strcmp(varbuf, "lookahead_dist"))    { lookahead_dist       = valuebuf; }
    else if (!strcmp(varbuf, "lookahead_time"))    { lookahead_time       = valuebuf; }
    else if (!strcmp(varbuf, "lookahead_kmax"))    { lookahead_kmax       = valuebuf; }
    else if (!strcmp(varbuf, "path_penalty"))      { path_penalty         = valuebuf; }
    else if (!strcmp(varbuf, "cone_penalty"))      { cone_penalty         = valuebuf; }
    else if (!strcmp(varbuf, "car_penalty"))       { car_penalty          = valuebuf; }
    else if (!strcmp(varbuf, "motor_gain"))        { motor_gain           = valuebuf; }
    else if (!strcmp(varbuf, "motor_kI"))          { motor_kI             = valuebuf; }
    else if (!strcmp(varbuf, "servo_rate"))        { servo_rate           = valuebuf; }
    else if (!strcmp(varbuf, "servo_offset"))      { servo_offset         = valuebuf; }
    else if (!strcmp(varbuf, "servo_kI"))          { servo_kI             = valuebuf; }
    else if (!strcmp(varbuf, "servo_min"))         { servo_min            = valuebuf; }
    else if (!strcmp(varbuf, "servo_max"))         { servo_max            = valuebuf; }
    else { printf("driverconf.txt: ignoring unknown variable %s\n", varbuf); }
  }
  fclose(fp);
  return true;
}

int DriverConfig::SerializedSize() const {
  return 8 + sizeof(*this);
}

int DriverConfig::Serialize(uint8_t *buf, int buflen) {
  int siz = SerializedSize();
  memcpy(buf, "cfg1", 4);
  memcpy(buf+4, &siz, 4);
  memcpy(buf+8, this, sizeof(*this));
  return siz;
}
