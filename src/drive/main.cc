#include <fenv.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "drive/driver.h"
#include "hw/cam/cam.h"
#include "hw/car/car.h"
#include "hw/gpio/i2c.h"
#include "hw/imu/imu.h"
#include "hw/input/js.h"
#include "inih/cpp/INIReader.h"
#include "io/flushthread.h"
#include "localization/ceiltrack/ceiltrack.h"
#include "ui/display.h"

Driver *driver_ = NULL;
void handle_sigint(int signo) {
  if (driver_) driver_->Quit();
}

int main(int argc, char *argv[]) {
  I2C i2c;
  CarHW *carhw;
  IMU *imu;
  UIDisplay display;
  FlushThread flush_thread;
  CeilingTracker ceiltrack;
  ObstacleDetector obstacledetector;

  signal(SIGINT, handle_sigint);

  feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW);

  INIReader ini("cycloid.ini");
  {
    int inierr = ini.ParseError();
    if (inierr != 0) {
      if (inierr == -1) {
        fprintf(stderr, "please create cycloid.ini!\n");
      } else {
        fprintf(stderr, "error loading cycloid.ini on line %d\n", inierr);
      }
      return 1;
    }
  }

  int fps = ini.GetInteger("camera", "fps", 30);

  if (!flush_thread.Init()) {
    return 1;
  }

  if (!Camera::Init(640, 480, fps)) return 1;

  JoystickInput js;

  if (!i2c.Open()) {
    fprintf(stderr, "need to enable i2c in raspi-config, probably\n");
    return 1;
  }

  bool has_joystick = false;
  if (js.Open(ini)) {
    has_joystick = true;
  } else {
    fprintf(stderr, "joystick not detected, but continuing anyway!\n");
  }

  imu = IMU::GetI2CIMU(i2c, ini);
  if (!imu || !imu->Init()) {
    fprintf(stderr, "unable to connect to IMU; aborting\n");
    return 1;
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  fprintf(stderr, "%ld.%06ld camera on @%d fps\n", tv.tv_sec, tv.tv_usec, fps);

  gettimeofday(&tv, NULL);
  fprintf(stderr, "%ld.%06ld started camera\n", tv.tv_sec, tv.tv_usec);

  carhw = CarHW::GetCar(&i2c, ini);
  if (!carhw || !carhw->Init()) {
    fprintf(stderr, "failed to init car hardware\n");
    return 1;
  }

  // FIXME(a1k0n): INI
  bool has_display = true;
  if (!display.Init()) {
    fprintf(stderr,
            "run this:\n"
            "sudo modprobe fbtft_device name=adafruit22a rotate=90\n"
            "running without display support for now\n");
    has_display = false;
  }

  driver_ = new Driver(&flush_thread, imu, has_joystick ? &js : NULL,
                       has_display ? &display : NULL);

  if (!driver_->Init(ini)) {
    return 1;
  }

  if (!Camera::StartRecord(driver_)) {
    return 1;
  }

  carhw->RunMainLoop(driver_);

  Camera::StopRecord();
}
