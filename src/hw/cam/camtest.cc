#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "cam/cam.h"

volatile bool done = false;

void handle_sigint(int signo) { done = true; }

class Recorder: public CameraReceiver {
 public:
  Recorder() { output_file_ = NULL; frame_ = 0; }

  bool Init(const char *fname) {
    if (!strcmp(fname, "-")) {
      output_file_ = stdout;
      setbuf(stdout, NULL);
    } else {
      output_file_ = fopen(fname, "wb");
    }
    if (!output_file_) {
      perror(fname);
      return false;
    }
    return true;
  }

  ~Recorder() {
    if (output_file_) fclose(output_file_);
  }

  void OnFrame(uint8_t *buf, size_t length) {
    struct timeval t;
    gettimeofday(&t, NULL);
    fwrite(&t.tv_sec, sizeof(t.tv_sec), 1, output_file_);
    fwrite(&t.tv_usec, sizeof(t.tv_usec), 1, output_file_);
    fwrite(buf, 1, length, output_file_);
    // fprintf(stderr, "%d.%06d frame %d\n", t.tv_sec, t.tv_usec, frame_++);
  }

 private:
  FILE *output_file_;
  int frame_;
};

int main(int argc, char *argv[]) {
  signal(SIGINT, handle_sigint);

  if (argc < 2) {
    fprintf(stderr, "%s [output.yuv] [fps]\n", argv[0]);
    return 1;
  }

  int fps = 20;

  if (argc > 2) {
    fps = atoi(argv[2]);
    if (fps == 0) {
      fprintf(stderr, "invalid fps %d\n", fps);
      return 1;
    }
  }

  if (!Camera::Init(320, 240, fps))
    return 1;

  struct timeval t;
  gettimeofday(&t, NULL);
  fprintf(stderr, "%d.%06d camera on @%d fps\n", t.tv_sec, t.tv_usec, fps);

  Recorder r;
  if (!r.Init(argv[1]))
    return 1;

  sleep(1);  // wait one second for auto-exposure to come up to speed

  if (!Camera::StartRecord(&r))
    return 1;

  gettimeofday(&t, NULL);
  fprintf(stderr, "%d.%06d started recording\n", t.tv_sec, t.tv_usec);

  while (!done) {
    usleep(100000);
  }

  Camera::StopRecord();
}
