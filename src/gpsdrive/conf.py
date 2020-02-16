configitems = '''\
speed limit      3.00
Ax limit         8.00
Ay limit        12.00
servo rate       1.00
servo offset     0.00
servo kI         0.20
servo min       -1.00
servo max        1.00
'''.split('\n')[:-1]


def main():
    conf = []

    h = open('config.h', 'w')
    h.write('''#ifndef GPSDRIVE_CONFIG_H_
#define GPSDRIVE_CONFIG_H_

// AUTO-GENERATED! Edit conf.py instead.

#include <stdint.h>

class DriverConfig {
 public:
''')
    for c in configitems:
        c = c.strip().split()
        default = int(float(c[-1]) * 100)
        varname = '_'.join(c[:-1])
        desc = ' '.join(c[:-1])
        conf.append((varname, desc, default))
        h.write('  int16_t %s;\n' % varname)
    h.write('''
  DriverConfig() {
    // Default values
''')
    for varname, desc, default in conf:
        h.write('    %-20s = %d;\n' % (varname, default))
    h.write('''\
  }

  static const char *confignames[];
  static const int N_CONFIGITEMS;

  bool Load();
  bool Save();
  int SerializedSize() const;
  int Serialize(uint8_t *buf, int buflen);
};

#endif  // GPSDRIVE_CONFIG_H_
''')
    h.close()

    cc = open('config.cc', 'w')
    cc.write('''// AUTO-GENERATED! Edit conf.py instead.

#include <stdio.h>
#include <string.h>
#include "drive/config.h"

const char *DriverConfig::confignames[] = {
''')
    for varname, desc, default in conf:
        cc.write('  "%s",\n' % desc)
    cc.write('''\
};

const int DriverConfig::N_CONFIGITEMS = %d;

bool DriverConfig::Save() {
  FILE *fp = fopen("driverconf.txt", "w");
  if (!fp) {
    perror("driverconf.txt");
    return false;
  }
''' % len(conf))
    for varname, desc, default in conf:
        cc.write('  fprintf(fp, "%-20s %%d\\n", %s);\n' % (varname, varname))
    cc.write('''
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
''')
    prefix = "     "
    for varname, desc, default in conf:
        cc.write('    %sif (!strcmp(varbuf, "%-20s { %-20s = valuebuf; }\n' % (prefix, varname + '"))', varname))
        prefix = "else "
    cc.write('''\
    else { printf("driverconf.txt: ignoring unknown variable %s\\n", varbuf); }
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
''')
    cc.close()


if __name__ == '__main__':
    main()
