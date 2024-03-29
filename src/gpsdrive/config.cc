// AUTO-GENERATED! Edit conf.py instead.

#include <stdio.h>
#include <string.h>
#include "gpsdrive/config.h"

const char *DriverConfig::confignames[] = {
  "speed limit",
  "Ax limit",
  "Ay limit",
  "motor gain",
  "motor kI",
  "servo rate",
  "servo offset",
  "servo kP",
  "servo kI",
  "servo min",
  "servo max",
  "steering kpy",
  "steering kvy",
  "lookahead",
  "deadreckon time",
  "cfilter cutoff",
};

const int DriverConfig::N_CONFIGITEMS = 16;

bool DriverConfig::Save() {
  FILE *fp = fopen("driverconf.txt", "w");
  if (!fp) {
    perror("driverconf.txt");
    return false;
  }
  fprintf(fp, "speed_limit          %d\n", speed_limit);
  fprintf(fp, "Ax_limit             %d\n", Ax_limit);
  fprintf(fp, "Ay_limit             %d\n", Ay_limit);
  fprintf(fp, "motor_gain           %d\n", motor_gain);
  fprintf(fp, "motor_kI             %d\n", motor_kI);
  fprintf(fp, "servo_rate           %d\n", servo_rate);
  fprintf(fp, "servo_offset         %d\n", servo_offset);
  fprintf(fp, "servo_kP             %d\n", servo_kP);
  fprintf(fp, "servo_kI             %d\n", servo_kI);
  fprintf(fp, "servo_min            %d\n", servo_min);
  fprintf(fp, "servo_max            %d\n", servo_max);
  fprintf(fp, "steering_kpy         %d\n", steering_kpy);
  fprintf(fp, "steering_kvy         %d\n", steering_kvy);
  fprintf(fp, "lookahead            %d\n", lookahead);
  fprintf(fp, "deadreckon_time      %d\n", deadreckon_time);
  fprintf(fp, "cfilter_cutoff       %d\n", cfilter_cutoff);

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
    else if (!strcmp(varbuf, "Ax_limit"))          { Ax_limit             = valuebuf; }
    else if (!strcmp(varbuf, "Ay_limit"))          { Ay_limit             = valuebuf; }
    else if (!strcmp(varbuf, "motor_gain"))        { motor_gain           = valuebuf; }
    else if (!strcmp(varbuf, "motor_kI"))          { motor_kI             = valuebuf; }
    else if (!strcmp(varbuf, "servo_rate"))        { servo_rate           = valuebuf; }
    else if (!strcmp(varbuf, "servo_offset"))      { servo_offset         = valuebuf; }
    else if (!strcmp(varbuf, "servo_kP"))          { servo_kP             = valuebuf; }
    else if (!strcmp(varbuf, "servo_kI"))          { servo_kI             = valuebuf; }
    else if (!strcmp(varbuf, "servo_min"))         { servo_min            = valuebuf; }
    else if (!strcmp(varbuf, "servo_max"))         { servo_max            = valuebuf; }
    else if (!strcmp(varbuf, "steering_kpy"))      { steering_kpy         = valuebuf; }
    else if (!strcmp(varbuf, "steering_kvy"))      { steering_kvy         = valuebuf; }
    else if (!strcmp(varbuf, "lookahead"))         { lookahead            = valuebuf; }
    else if (!strcmp(varbuf, "deadreckon_time"))   { deadreckon_time      = valuebuf; }
    else if (!strcmp(varbuf, "cfilter_cutoff"))    { cfilter_cutoff       = valuebuf; }
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
