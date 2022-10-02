// AUTO-GENERATED! Edit conf.py instead.

#include <stdio.h>
#include <string.h>
#include "drive/config.h"

const char *DriverConfig::confignames[] = {
  "speed limit",
  "throttle cap",
  "throttle slew",
  "throttle bias",
  "pi thr scale",
  "pi brake scale",
  "pi steer scale",
  "pi steer_v scale",
  "pi v scale",
  "orange thresh",
  "black thresh",
  "servo offset",
  "servo min",
  "servo max",
};

const int DriverConfig::N_CONFIGITEMS = 14;

bool DriverConfig::Save() {
  FILE *fp = fopen("driverconf.txt", "w");
  if (!fp) {
    perror("driverconf.txt");
    return false;
  }
  fprintf(fp, "speed_limit          %d\n", speed_limit);
  fprintf(fp, "throttle_cap         %d\n", throttle_cap);
  fprintf(fp, "throttle_slew        %d\n", throttle_slew);
  fprintf(fp, "throttle_bias        %d\n", throttle_bias);
  fprintf(fp, "pi_thr_scale         %d\n", pi_thr_scale);
  fprintf(fp, "pi_brake_scale       %d\n", pi_brake_scale);
  fprintf(fp, "pi_steer_scale       %d\n", pi_steer_scale);
  fprintf(fp, "pi_steer_v_scale     %d\n", pi_steer_v_scale);
  fprintf(fp, "pi_v_scale           %d\n", pi_v_scale);
  fprintf(fp, "orange_thresh        %d\n", orange_thresh);
  fprintf(fp, "black_thresh         %d\n", black_thresh);
  fprintf(fp, "servo_offset         %d\n", servo_offset);
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
    else if (!strcmp(varbuf, "throttle_cap"))      { throttle_cap         = valuebuf; }
    else if (!strcmp(varbuf, "throttle_slew"))     { throttle_slew        = valuebuf; }
    else if (!strcmp(varbuf, "throttle_bias"))     { throttle_bias        = valuebuf; }
    else if (!strcmp(varbuf, "pi_thr_scale"))      { pi_thr_scale         = valuebuf; }
    else if (!strcmp(varbuf, "pi_brake_scale"))    { pi_brake_scale       = valuebuf; }
    else if (!strcmp(varbuf, "pi_steer_scale"))    { pi_steer_scale       = valuebuf; }
    else if (!strcmp(varbuf, "pi_steer_v_scale"))  { pi_steer_v_scale     = valuebuf; }
    else if (!strcmp(varbuf, "pi_v_scale"))        { pi_v_scale           = valuebuf; }
    else if (!strcmp(varbuf, "orange_thresh"))     { orange_thresh        = valuebuf; }
    else if (!strcmp(varbuf, "black_thresh"))      { black_thresh         = valuebuf; }
    else if (!strcmp(varbuf, "servo_offset"))      { servo_offset         = valuebuf; }
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
