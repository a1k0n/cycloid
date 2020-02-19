#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <termios.h>
#include <unistd.h>

#include "gps/ubx.h"

const char ubx_port[] = "/dev/serial0";
#define startup_ioctl_baud B9600
#define runtime_baudrate 115200
#define runtime_ioctl_baud B115200

namespace {

void nmea_sendmsg(int fd, const char *msg) {
  int i, cksum = 0, len = strlen(msg);
  char footer[6];
  for (i = 1; i < len; i++)
    cksum ^= msg[i];
  snprintf(footer, sizeof(footer), "*%02x\r\n", cksum);
  write(fd, msg, len);
  write(fd, footer, 5);
}

void ubx_sendmsg(int fd, int msg_class, int msg_id, uint8_t *msg, int msg_len) {
  static uint8_t header[6] = {0xb5, 0x62}, footer[2];
  static struct iovec iov[3] = {{header, 6}, {0, 0}, {footer, 2}};
  uint8_t cka, ckb;
  int i;
  iov[1].iov_base = msg;
  iov[1].iov_len = msg_len;
  header[2] = msg_class;
  cka = ckb = msg_class;
  header[3] = msg_id;
  cka += msg_id;
  ckb += cka;
  header[4] = msg_len & 0xff;
  cka += header[4];
  ckb += cka;
  header[5] = msg_len >> 8;
  cka += header[5];
  ckb += cka;
  for (i = 0; i < msg_len; i++) {
    cka += msg[i];
    ckb += cka;
  }
  footer[0] = cka;
  footer[1] = ckb;
  int len = writev(fd, iov, 3);
#if 0
  for (i = 0; i < 6; i++)
    printf("%02x ", header[i]);
  printf("| ");
  for (i = 0; i < msg_len; i++)
    printf("%02x ", msg[i]);
  printf("| ");
  printf("%02x %02x -- wrote %d\n", footer[0], footer[1], len);
#endif
  if (len == -1) {
    perror("ubx_sendmsg: writev");
  }
}

void ubx_enable_periodic(int fd, uint8_t msgclass, uint8_t msgid, uint8_t enable) {
  fprintf(stderr, "%sabling (%d,%d)\n", enable ? "en" : "dis",
          msgclass, msgid);
  // now, CFG-MSG and set NAV-SOL output on every epoch
  uint8_t cfg_msg[] = {
    msgclass, msgid,       // class/id of NAV-POSLLH
    0, enable, 0, 0, 0, 0  // output once per epoch on port 1
  };
  ubx_sendmsg(fd, 6, 1, cfg_msg, 8);
}

void process_msg(int fd, int msg_class, int msg_id, uint8_t *msgbuf,
                 int msg_length, void (*on_pvt)(const nav_pvt &)) {
  int i;
  switch ((msg_class << 8) + msg_id) {
    case 0x0101:  // NAV-POSECEF
    {
      struct nav_posecef *navmsg = (struct nav_posecef *)msgbuf;
      // on_ecef(navmsg);
    } break;
    case 0x0102:  // NAV-POSLLH (ignored)
      break;
    case 0x0107:  // NAV-PVT
    {
      const struct nav_pvt *navmsg = (struct nav_pvt *)msgbuf;
      on_pvt(*navmsg);
      break;
    }
    case 0x0501:  // ACK
    case 0x0500:  // NAK
      fprintf(stderr, "%s (%d,%d)\n", msg_id == 1 ? "ack" : "nak", msgbuf[0],
              msgbuf[1]);
      break;
    default:
      fprintf(stderr, "read msg (%02x,%02x): ", msg_class, msg_id);
      for (i = 0; i < msg_length; i++) fprintf(stderr, "%02x ", msgbuf[i]);
      fprintf(stderr, "\n");
      // diable unknown messages?
      // ubx_enable_periodic(fd, msg_class, msg_id, 0);
  }
}

void init_ubx_protocol(int fd) {
  struct termios tios;
  tcgetattr(fd, &tios);
  // disable flow control and all that, and ignore break and parity errors
  tios.c_iflag = IGNBRK | IGNPAR;
  tios.c_oflag = 0;
  tios.c_lflag = 0;
  cfsetspeed(&tios, startup_ioctl_baud);
  tcsetattr(fd, TCSAFLUSH, &tios);
  // the serial port has a brief glitch once we turn it on which generates a
  // start bit; sleep for 1ms to let it settle
  usleep(10000);

  write(fd, "\r\n\r\n", 4);

  // now send baudrate-changing message
  char nmeamsg[256];
  snprintf(nmeamsg, sizeof(nmeamsg),
           "$PUBX,41,1,0007,0001,%d,0", runtime_baudrate);
  nmea_sendmsg(fd, nmeamsg);

  // add some guard time before and after changing baud rates
  usleep(100000);
  cfsetspeed(&tios, runtime_ioctl_baud);
  tcsetattr(fd, TCSADRAIN, &tios);
}

}  // empty namespace

int ubx_open() {
  int fd = open(ubx_port, O_RDWR);
  if (fd == -1) {
    perror(ubx_port);
    return -1;
  }
  init_ubx_protocol(fd);
  close(fd);
  fd = open(ubx_port, O_RDWR);
  if (fd == -1) {
    perror(ubx_port);
    return -1;
  }

  // ubx_sendmsg(fd, 6, 6, NULL, 0);

  // ubx-cfg-rate
  uint8_t msgbuf[6] = {100, 0, 1, 0, 0, 0};  // 100ms measurements, one solution per measurement, UTC timeref
  ubx_sendmsg(fd, 6, 8, msgbuf, 6);

  // ubx_enable_periodic(fd, 1, 2, 1);  // enable NAV-POSLLH
  // ubx_enable_periodic(fd, 1, 1, 1);  // enable NAV-POSECEF
  ubx_enable_periodic(fd, 1, 7, 1);  // enable UBX-NAV-PVT

  return fd;
}

void ubx_read_loop(int fd, void (*on_pvt)(const nav_pvt &)) {
  uint8_t buf[512];
  static uint8_t msgbuf[512];
  static int read_state = 0;
  static unsigned msg_length = 0, msg_ptr = 0;
  static int msg_cls = 0, msg_id = 0;
  static uint8_t msg_cka = 0, msg_ckb = 0;
  for (;;) {
    int len = read(fd, buf, sizeof(buf));
    int i;
    if (len == -1) {
      perror("read");
      return;
    }
    for (i = 0; i < len; i++) {
      // printf("%02x ", buf[i]);
      if (read_state > 1 && read_state < 7) {
        msg_cka += buf[i];
        msg_ckb += msg_cka;
      }
      switch (read_state) {
        case 0:
          if (buf[i] == 0xb5) {
            read_state++;
          } else {
            putchar(buf[i]);
            fflush(stdout);
          }
          break;
        case 1:
          if (buf[i] == 0x62)
            read_state++;
          else if (buf[i] != 0xb5)
            read_state = 0;
          msg_cka = 0;
          msg_ckb = 0;
          break;
        case 2:
          msg_cls = buf[i];
          read_state++;
          break;
        case 3:
          msg_id = buf[i];
          read_state++;
          break;
        case 4:
          msg_length = buf[i];
          read_state++;
          break;
        case 5:
          msg_length += buf[i] << 8;
          read_state++;
          msg_ptr = 0;
          if (msg_length > sizeof(msgbuf)) {
            fprintf(stderr,
                    "discarding (%02x,%02x) message of length %d "
                    "(buffer is %d)\n",
                    msg_cls, msg_id, msg_length, sizeof(msgbuf));
            read_state = 0;
          }
          if (msg_length == 0) read_state++;  // skip payload read
          break;
        case 6:
          msgbuf[msg_ptr++] = buf[i];
          if (msg_ptr == msg_length) read_state++;
          break;
        case 7:
          if (msg_cka != buf[i]) {
            fprintf(stderr,
                    "discarding (%02x,%02x) message; "
                    "cka mismatch (got %02x calc'd %02x)\n",
                    msg_cls, msg_id, buf[i], msg_cka);
            read_state = 0;
          } else {
            read_state++;
          }
          break;
        case 8:
          if (msg_ckb != buf[i]) {
            fprintf(stderr,
                    "discarding (%02x,%02x) message; "
                    "cka mismatch (got %02x calc'd %02x)\n",
                    msg_cls, msg_id, buf[i], msg_cka);
          } else {
            process_msg(fd, msg_cls, msg_id, msgbuf, msg_length, on_pvt);
          }
          read_state = 0;
          break;
      }
    }
    // printf("\n");
  }
}
