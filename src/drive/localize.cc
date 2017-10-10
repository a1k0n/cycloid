#include <math.h>
#include <Eigen/Dense>

#include "drive/localize.h"

using Eigen::VectorXf;
using Eigen::MatrixXf;

// FIXME(asloane): maps are hardcoded for now; will want to make them loadable

#define HOME_TRACK 1

#ifdef HOME_TRACK
// for track at home
static const int MAP_NPOINTS = 96;
static const float MAP_ENTRIES_PER_METER = 100.0 / 19.8059962878;

static const float track_k[MAP_NPOINTS] = {
#include "home_track_k.txt"
};

static const float raceline_k[MAP_NPOINTS] = {
#include "home_raceline_k.txt"
};

static const float raceline_y[MAP_NPOINTS] = {
#include "home_raceline_y.txt"
};

static const float raceline_psi[MAP_NPOINTS] = {
#include "home_raceline_psi.txt"
};

static const float raceline_v[MAP_NPOINTS] = {
#include "home_raceline_v.txt"
};

#endif

MapLocalizer::MapLocalizer(): p_s_(MAP_NPOINTS) {
  ResetUnknown();
}

void MapLocalizer::ResetToStart() {
  p_s_ = VectorXf::Zero(MAP_NPOINTS);
  p_s_[0] = 1;
}

void MapLocalizer::ResetUnknown() {
  p_s_ = VectorXf::Ones(MAP_NPOINTS) / MAP_NPOINTS;
}

void MapLocalizer::Predict(const Eigen::VectorXf &x,
    const Eigen::MatrixXf &P,
    float Delta_t) {

  // Code generated from sympy assuming state variables in x are:
  float v = x[0];
  float y_e = x[2];
  float psi_e = x[3];
  float kappa = x[4];

  float x0 = cosf(psi_e);
  float x1 = 1 - kappa*y_e;
  float x2 = Delta_t/x1;
  float x3 = x0*x2;
  float x4 = Delta_t*v*x0/(x1*x1);

  // ds and variance of ds projected from state estimate
  float ds = v * x3;
  VectorXf J(5);
  J << x3, 0, kappa*x4, -v*x2*sinf(psi_e), x4*y_e;
  float ds_var = J.transpose() * P.block<5, 5>(0, 0) * J;

  ds *= MAP_ENTRIES_PER_METER;
  ds_var *= MAP_ENTRIES_PER_METER * MAP_ENTRIES_PER_METER;
  float dsi = floorf(ds);
  float dsf = ds - dsi;
  float ds_prec = 1.0 / ds_var;

  // compute a circular gaussian convolution
  // this is an N^2 operation, but the least we can do is precompute the kernel
  // which we apply, rotated, everywhere
  // this kernel isn't necessarily normalized to 1, but we will normalize later
  VectorXf kernel(MAP_NPOINTS);
  for (int i = 0; i < MAP_NPOINTS; i++) {
    kernel[i] = expf(-powf(fmin(i + dsi, MAP_NPOINTS - i - dsi), 2) * ds_prec);
  }

  VectorXf ps_new(MAP_NPOINTS);
  double psum = 0;
  for (int i = 0; i < MAP_NPOINTS; i++) {
    // 789 . 0123456 *
    // 012 . 3456789
    ps_new[i] = p_s_.head(i).dot(kernel.tail(i))
      + p_s_.tail(MAP_NPOINTS - i).dot(kernel.head(MAP_NPOINTS - i));
    psum += ps_new[i];
  }
  ps_new /= psum;
  p_s_ = (1-dsf)*ps_new;
  p_s_.tail(MAP_NPOINTS-1) += dsf * ps_new.head(MAP_NPOINTS-1);
  p_s_[0] += dsf * ps_new[MAP_NPOINTS-1];
}

void MapLocalizer::Update(float k, float prec) {
  Eigen::Map<const VectorXf> tk(track_k, MAP_NPOINTS);
  Eigen::VectorXf l = -(tk.array() - k).square() * prec;
  l = (l.array() - l.maxCoeff()).exp() * p_s_.array();

  float s = l.sum();
  if (s == 0) {
    ResetUnknown();
    return;
  }
  p_s_ = l / s;
}

float MapLocalizer::GetRacelineVelocity() const {
  return p_s_.dot(Eigen::Map<const VectorXf>(raceline_v, MAP_NPOINTS));
}

float MapLocalizer::GetRacelineOffset() const {
  return p_s_.dot(Eigen::Map<const VectorXf>(raceline_y, MAP_NPOINTS));
}

float MapLocalizer::GetRacelineAngle() const {
  return p_s_.dot(Eigen::Map<const VectorXf>(raceline_psi, MAP_NPOINTS));
}

float MapLocalizer::GetRacelineCurvature() const {
  return p_s_.dot(Eigen::Map<const VectorXf>(raceline_k, MAP_NPOINTS));
}


