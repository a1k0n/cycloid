#include "gpsdrive/gpsdrive.h"

#include <sys/time.h>

#include <Eigen/Dense>
#include <algorithm>
#include <string>

#include "gpsdrive/config.h"
#include "hw/car/car.h"
#include "hw/gps/ubx.h"
#include "hw/imu/imu.h"
#include "hw/imu/mag.h"
#include "hw/input/js.h"
#include "inih/cpp/INIReader.h"
#include "inih/ini.h"
#include "ui/display.h"

float clamp(float x, float min, float max) {
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}

GPSDrive::GPSDrive(FlushThread *ft, IMU *imu, Magnetometer *mag,
                   JoystickInput *js, UIDisplay *disp)
    : flush_thread_(ft),
      imu_(imu),
      mag_(mag),
      js_(js),
      display_(disp),
      gyro_last_(0, 0, 0),
      gyro_bias_(0, 0, 0),
      gps_v_(0, 0, 0) {
  done_ = false;
  record_fp_ = NULL;
  js_throttle_ = 0;
  js_steering_ = 0;
  config_item_ = 0;
  ierr_k_ = 0;
  ierr_v_ = 0;
  brake_count_ = 0;
  last_v_ = 0;
  last_w_ = 0;
  last_target_k_ = 0;
  lat_ = lon_ = 0;
  numSV_ = 0;

  ye_ = psie_ = k_ = 0;
  autodrive_k_ = 0;
  autodrive_v_ = 0;

  control_hist_ptr_ = 0;
  state_hist_ptr_ = 0;

  autodrive_ = false;
  x_down_ = y_down_ = false;
  pthread_mutex_init(&record_mut_, NULL);
}

void* GPSDrive::gpsThread(void* arg) {
  GPSDrive *drive = (GPSDrive*)arg;
  fprintf(stderr, "GPS receive thread started\n");
  ubx_read_loop(drive->ubx_fd_, drive);
  return NULL;
}

bool GPSDrive::Init(const INIReader &ini) {
  if (config_.Load()) {
    fprintf(stderr, "Loaded driver configuration\n");
  }

  ubx_fd_ = ubx_open();
  if (ubx_fd_ == -1) {
    return false;
  }

  if (pthread_create(&gps_thread_, NULL, GPSDrive::gpsThread, (void*) this)) {
    perror("pthread_create");
    return false;
  }

  ref_lat_ = ini.GetInteger("nav", "reflat", 0);
  ref_lon_ = ini.GetInteger("nav", "reflon", 0);
  if (ref_lat_ == 0 || ref_lon_ == 0) {
    fprintf(stderr, "Please provide [nav] reflat and reflon in cycloid.ini\n");
    fprintf(stderr, "note: they are integers, w/ 7 decimal places\n");
    return false;
  }

  std::string trackfile = ini.GetString("nav", "track", "track.txt");
  if (!raceline_.LoadTrack(trackfile.c_str())) {
    return false;
  }

  // compute meters / 1e-7 degree on WGS84 ellipsoid
  // this is an approximation that assumes 0 altitude
  double invf = 298.257223563;                  // WGS84 inverse flattening
  double a = 6378137.0;                         // meters
  double b = a * (1 - 1/invf);
  double lat = ref_lat_ * M_PI * 1e-7 / 180.0;  // lat/lon in radians
  double clat = cos(lat);

  mscale_lat_ = b * M_PI / 180.0e7;
  mscale_lon_ = a * clat * M_PI / 180.0e7;

  // draw UI screen
  UpdateDisplay();
  display_->UpdateStatus("GPSDrive started.");

  return true;
}

GPSDrive::~GPSDrive() {}

void GPSDrive::UpdateControls(float in_throttle, float in_steering,
                              bool radio_safe, const StateObservation &obs,
                              float dt, ControlOutput *out) {
  const float srv_off = 0.001f * config_.servo_offset;
  const float srv_ratio =
      100.f / (config_.servo_rate == 0 ? 100 : config_.servo_rate);
  const float srv_kI = 0.01f * config_.servo_kI;

  float u_s = clamp(in_steering + srv_off, config_.servo_min * 0.01f,
                    config_.servo_max * 0.01f);

  if (!autodrive_ && in_throttle <= 0.05) {
    out->Set(2, in_throttle, u_s);
    if (in_throttle < -0.05) {
      brake_count_ = 10;
    }
    ierr_v_ = 0;
    ierr_k_ = 0;
    last_v_ = obs.vx;
    last_w_ = obs.w;
    return;
  }

  if (autodrive_ && !radio_safe) {
    if (in_throttle < -0.5) {
      autodrive_ = false;
    }
    out->Set(2, -1, 0);
    return;
  }

  float max_throttle = 1.0;
  float target_v = config_.speed_limit * 0.01 * clamp(in_throttle, 0.f, 1.f);
  // float target_k = u_steering;
  float target_k = -in_steering * 1;

  // if autodriving, override target velocity and steering
  if (autodrive_) {
    // joystick control only goes up to 0.8, so give it a little boost
    max_throttle = in_throttle * 1.3;
    target_v = clamp(autodrive_v_, 0, config_.speed_limit * 0.01f);
    target_k = autodrive_k_;
  }

  if (obs.vx > 1.0) {
    // use a delayed target_k here so we're comparing against the yaw it
    // induced last control frame

    // 1 = t*kerr*srv_kI
    // t = 1/(kerr * srv_kI)
    // set srv_kI to, like, 10 or so except w is a bit noisy
    // i think it's already set to 2?
    float kerr = last_target_k_ - obs.w / obs.vx;
    ierr_k_ = clamp(ierr_k_ + dt * srv_kI * kerr, -0.5f, 0.5f);
  } else {
    ierr_k_ = 0;
  }

  u_s = clamp((target_k + ierr_k_) * srv_ratio + srv_off,
              config_.servo_min * 0.01f, config_.servo_max * 0.01f);

  float vgain = 0.01 * config_.motor_gain;
  float kI = 0.01 * config_.motor_kI;
  float verr = target_v - obs.vx;
  float u = vgain * verr + kI * (ierr_v_ + verr * dt);
  if (u > -1 && u < 1) {
    ierr_v_ += verr * dt;
  }
  if (target_v < obs.vx * 0.9) {
    u = clamp(u, -1.f, max_throttle);
  } else {
    u = clamp(u, 0.f, max_throttle);
  }
  out->Set(1, u, u_s);

  last_v_ = obs.vx;
  last_w_ = obs.w;
  last_target_k_ = target_k;
  if (u < -0.05) {
    brake_count_ = 10;
  }
}

bool GPSDrive::OnControlFrame(CarHW *car, float dt) {
  if (js_) {
    js_->ReadInput(this);
  }

  Eigen::Vector3f accel, gyro, mag;
  if (!imu_->ReadIMU(&accel, &gyro)) {
    fprintf(stderr, "imu read failure\n");
    accel = accel.Zero();
    gyro = gyro.Zero();
  } else {
    gyro_last_ = 0.95 * gyro_last_ + 0.05 * gyro;
  }
  gyro -= gyro_bias_;

  mag = mag.Zero();
  // disable magnetometer for now
#if 0
  if (!mag_->ReadMag(&mag)) {
    fprintf(stderr, "magnetometer read failure\n");
  }
  float MagN = mag.dot(MAGCALN);
  float MagE = mag.dot(MAGCALE);
  float renorm = sqrtf(MagN*MagN + MagE*MagE);
  MagN /= renorm;
  MagE /= renorm;
#else
  float MagN = 0, MagE = 0;
#endif

  bool radio_safe = false;  // runaway protection

  float controls[2] = {0, 0};
  float in_throttle = 0;
  float in_steering = 0;
  if (car->GetRadioInput(controls, 2)) {
    in_throttle = controls[0];
    in_steering = controls[1];
    if (in_throttle > 0.15) {
      radio_safe = true;
    }
  } else {
    in_throttle = js_throttle_ / 32768.0;
    in_steering = js_steering_ / 32760.0;
  }

  float ds = 0, wheelv = 0;
  float w = gyro[2];
  car->GetWheelMotion(&ds, &wheelv);

  float v = wheelv;
  if (brake_count_ > 0) {
    brake_count_--;
    // dumb assumption: we rapidly decay speed estimate when we hit the brake
    // so that we "pump" the brakes so we can see how fast we're going
    // v = last_v_ * 0.95;
    // this isn't going to work
    // v = last_v_ * 0.95;
    // we need to use GPS velocity here

    v = gps_v_.norm();
  }

  ControlOutput out;
  StateObservation obs;
  obs.vx = v;
  obs.w = gyro[2];
  UpdateControls(in_throttle, in_steering, radio_safe, obs, dt, &out);
  car->SetControls(out.leds, out.u_esc, out.u_servo);

  if (record_fp_ != NULL) {
    timeval tv;
    gettimeofday(&tv, NULL);
    pthread_mutex_lock(&record_mut_);
    fprintf(record_fp_,
            "%ld.%06ld control %f %f wheel %f %f imu %f %f %f %f %f %f mag %f "
            "%f %f windup_vk %f %f\n",
            tv.tv_sec, tv.tv_usec, out.u_esc, out.u_servo, ds, v, accel[0],
            accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1],
            mag[2], ierr_v_, ierr_k_);
    pthread_mutex_unlock(&record_mut_);
  }

  if (display_) {
    display_->UpdateDashboard(v, w, lon_, lat_, numSV_, gps_v_.norm(),
                              (lon_ - ref_lon_) * mscale_lon_,
                              (lat_ - ref_lat_) * mscale_lat_, MagN, MagE, ye_,
                              psie_, autodrive_k_, autodrive_v_);
  }

  return !done_;
}

void GPSDrive::OnNav(const nav_pvt &msg) {
  lat_ = msg.lat;
  lon_ = msg.lon;
  numSV_ = msg.numSV;
  gps_v_ = Eigen::Vector3f(msg.velN, msg.velE, msg.velD) * 0.001f;

  // lookup closest point on target trajectory
  // get east/north meters w.r.t. reference lat/lon
  float E = (lon_ - ref_lon_) * mscale_lon_;
  float N = (lat_ - ref_lat_) * mscale_lat_;
  float cx, cy, Nx, Ny, kl;
  raceline_.GetTarget(E, N, config_.lookahead, &cx, &cy, &Nx, &Ny, &k_, &kl);
  //  - compute track_ye, track_psie, track_k
  ye_ = (E - cx)*Nx + (N - cy)*Ny;

  // compute heading angle from GPS velocity
  // this is fraught with peril if we aren't moving very much; so if we're
  // stationary, assume we're pointing the right way
  float C = msg.velE;
  float S = msg.velN;
  float mag = sqrtf(C*C + S*S);
  psie_ = 0;
  float Cp = 1;  // cos(psi)
  float Sp = 0;  // sin(psi)
  if (mag > 300) {
    // if we're moving more than 0.3 m/s, then compute a proper psie
    C /= mag;
    S /= mag;
    Cp = -S * Nx + C * Ny;
    Sp = S * Ny + C * Nx;
    psie_ = atan2(Sp, Cp);
  }

  float omkye = 1 - k_ * ye_;
  if (fabsf(omkye) < 0.1) {
    omkye = 0.1 * (std::signbit(omkye) ? -1 : 1);
  }

  float Kpy = config_.steering_kpy * 0.01;
  float Kvy = config_.steering_kvy * 0.01;
  float Cpy = Cp / omkye;
  autodrive_k_ = Cpy*(ye_*Cpy*(-Kpy*Cp) + Sp*(k_*Sp - Kvy*Cp) + k_);

  float alimit = config_.Ay_limit * 0.01;
  float vmax = config_.speed_limit * 0.01;
  float kmin = alimit / (vmax*vmax);
  autodrive_v_ = sqrtf(alimit / std::max(kmin, fabsf(kl)));

#if 0
  printf("closest track point %f %f -> %f %f (%f %f)\n", cx, cy, cx / mscale_lon_ + ref_lon_,
         cy / mscale_lat_ + ref_lat_, mscale_lon_, mscale_lat_);
#endif

  if (record_fp_ != NULL) {
    timeval tv;
    gettimeofday(&tv, NULL);
    pthread_mutex_lock(&record_mut_);
    fprintf(record_fp_, "%ld.%06ld gps ", tv.tv_sec, tv.tv_usec);
    fprintf(record_fp_, "%04d-%02d-%02dT%02d:%02d:%02d.%09d ", msg.year,
            msg.month, msg.day, msg.hour, msg.min, msg.sec, msg.nano);
    fprintf(record_fp_,
        "fix:%d numSV:%d %d.%07d +-%dmm %d.%07d +-%dmm height %dmm "
        "vel %d %d %d +-%d mm/s "
        "heading motion %d.%05d vehicle %d +- %d.%05d\n",
        msg.fixType, msg.numSV, msg.lon / 10000000,
        std::abs(msg.lon) % 10000000, msg.hAcc, msg.lat / 10000000,
        std::abs(msg.lat) % 10000000, msg.vAcc, msg.height, msg.velN, msg.velE,
        msg.velD, msg.sAcc, msg.headMot / 100000,
        std::abs(msg.headMot) % 100000, msg.headVeh, msg.headAcc / 100000,
        msg.headAcc % 100000);
    fprintf(record_fp_, "%ld.%06ld nav %0.4f %0.4f %f kv %0.5f %0.4f\n",
            tv.tv_sec, tv.tv_usec, ye_, psie_, k_, autodrive_k_, autodrive_v_);
    pthread_mutex_unlock(&record_mut_);
  }
}

void GPSDrive::OnDPadPress(char direction) {
  int16_t *value = ((int16_t *)&config_) + config_item_;
  switch (direction) {
    case 'U':
      --config_item_;
      if (config_item_ < 0) config_item_ = DriverConfig::N_CONFIGITEMS - 1;
      fprintf(stderr, "\n");
      break;
    case 'D':
      ++config_item_;
      if (config_item_ >= DriverConfig::N_CONFIGITEMS) config_item_ = 0;
      fprintf(stderr, "\n");
      break;
    case 'L':
      if (y_down_) {
        *value -= 100;
      } else if (x_down_) {
        *value -= 10;
      } else {
        --*value;
      }
      break;
    case 'R':
      if (y_down_) {
        *value += 100;
      } else if (x_down_) {
        *value += 10;
      } else {
        ++*value;
      }
      break;
  }
  UpdateDisplay();
}

void GPSDrive::OnButtonPress(char button) {
  switch (button) {
    case '+':  // start button
      StartRecording();
      break;
    case '-':  // stop button
      StopRecording();
      break;
    case 'B':
      if (config_.Load()) {
        fprintf(stderr, "config loaded\n");
        int16_t *values = ((int16_t *)&config_);
        if (display_) {
          display_->UpdateConfig(DriverConfig::confignames,
                                 DriverConfig::N_CONFIGITEMS, config_item_,
                                 values);
          display_->UpdateStatus("config loaded", 0xffff);
        }
      }
      fprintf(stderr, "reset kalman filter\n");
      break;
    case 'A':
      if (config_.Save()) {
        fprintf(stderr, "config saved\n");
        if (display_) display_->UpdateStatus("config saved", 0xffff);
      }
      break;
    case 'X':
      x_down_ = true;
      break;
    case 'Y':
      y_down_ = true;
      break;
    case 'H':  // home button: init to start line
      gyro_bias_ = gyro_last_;
      break;
    case 'L':
      if (autodrive_) {
        fprintf(stderr, "autodrive OFF\n");
        if (display_) display_->UpdateStatus("autodrive OFF", 0xffff);
      }
      autodrive_ = false;
      break;
    case 'R':
      if (!autodrive_) {
        fprintf(stderr, "autodrive ON\n");
        if (display_) display_->UpdateStatus("autodrive ON", 0xffff);
      }
      autodrive_ = true;
      break;
  }
}

void GPSDrive::OnButtonRelease(char button) {
  switch(button) {
    case 'X':
      x_down_ = false;
      break;
    case 'Y':
      y_down_ = false;
      break;
  }
}

void GPSDrive::OnAxisMove(int axis, int16_t value) {
  switch (axis) {
    case 1:  // left stick y axis
      js_throttle_ = -value;
      break;
    case 2:  // right stick x axis
      js_steering_ = value;
      break;
  }
}

void GPSDrive::StartRecording() {
  timeval tv;
  gettimeofday(&tv, NULL);

  if (record_fp_ != NULL) {
    return;
  }

  char fnamebuf[256];
  time_t start_time = time(NULL);
  struct tm start_time_tm;
  localtime_r(&start_time, &start_time_tm);
  strftime(fnamebuf, sizeof(fnamebuf), "gpsdrive-%Y%m%d-%H%M%S.log",
           &start_time_tm);
  record_fp_ = fopen(fnamebuf, "w");
  if (!record_fp_) {
    perror(fnamebuf);
  }

  printf("%ld.%06ld start recording %s\n", tv.tv_sec, tv.tv_usec, fnamebuf);
  display_->UpdateStatus(fnamebuf);
}

void GPSDrive::StopRecording() {
  if (record_fp_ == NULL) {
    return;
  }

  timeval tv;
  gettimeofday(&tv, NULL);

  pthread_mutex_lock(&record_mut_);
  fclose(record_fp_);
  record_fp_ = NULL;
  pthread_mutex_unlock(&record_mut_);

  printf("%ld.%06ld stop recording\n", tv.tv_sec, tv.tv_usec);
  display_->UpdateStatus("stop recording");
}

void GPSDrive::UpdateDisplay() {
  // hack because all config values are int16_t's in 1/100th steps
  int16_t *values = ((int16_t *)&config_);
  int16_t value = values[config_item_];
  // FIXME: does this work for negative values?
  fprintf(stderr, "%s %d.%02d\r", DriverConfig::confignames[config_item_], value / 100,
          value % 100);

  if (display_)
    display_->UpdateConfig(DriverConfig::confignames,
                           DriverConfig::N_CONFIGITEMS, config_item_, values);
}

void GPSDrive::Quit() {
  done_ = true;
  StopRecording();
}