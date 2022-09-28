#include <drive/pinet.h>

#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>

using Eigen::VectorXf;
using Eigen::MatrixXf;

// FIXME: this file format is brittle and dangerous
bool PiNetwork::Load(const char *fname) {
  FILE *fp = fopen(fname, "rb");
  if (!fp) {
    perror(fname);
    return false;
  }
  // Load up everything as a binary list of floats
  fread(emb, sizeof(float), HIDDEN*NEMBED, fp);
  fread(state_weight1, sizeof(float), 6*HIDDEN, fp);
  fread(state_bias1, sizeof(float), HIDDEN, fp);
  fread(linear_weight1, sizeof(float), HIDDEN * HIDDEN, fp);
  fread(linear_bias1, sizeof(float), HIDDEN, fp);
  fread(mu_weight, sizeof(float), HIDDEN * 2, fp);
  fread(mu_bias, sizeof(float), 2, fp);
  fread(logstd_weight, sizeof(float), HIDDEN * 2, fp);
  fread(logstd_bias, sizeof(float), 2, fp);
  fclose(fp);
  return true;
}

void PiNetwork::Action(int ix, float v, float w, float xl, float yl, float cl, float sl,
                       float tscale, float bscale, float sscale, float *u_throttle, float *u_steering) {
  VectorXf state(6);
  VectorXf linear(HIDDEN);

  // copy embed
  linear = Eigen::Map<VectorXf>(emb[ix], HIDDEN);

  state << v, w, xl, yl, cl, sl;

  // matmul state_weight1 with state
  linear.noalias() += Eigen::Map<MatrixXf>((float*)state_weight1, 6, HIDDEN).transpose() * state +
                      Eigen::Map<VectorXf>((float*)state_bias1, HIDDEN);

  // mish activation x*tanh(log1p(exp(x))), factored another way here
  for (int i = 0; i < HIDDEN; i++) {
    float e = 1.0f + expf(linear[i]);
    float e2 = e*e;
    linear[i] *= (e2-1.0f)/(e2+1.0f);
  }

  // linear 2
  linear = Eigen::Map<MatrixXf>((float *)linear_weight1, HIDDEN, HIDDEN).transpose() * linear +
           Eigen::Map<VectorXf>((float *)linear_bias1, HIDDEN);

  // mish 2
  for (int i = 0; i < HIDDEN; i++) {
    float e = 1.0f + expf(linear[i]);
    float e2 = e*e;
    linear[i] *= (e2-1.0f)/(e2+1.0f);
  }

  // mu layer
  VectorXf mu = Eigen::Map<MatrixXf>((float *)mu_weight, HIDDEN, 2).transpose() * linear +
                Eigen::Map<VectorXf>((float *)mu_bias, 2);

  if (mu[0] < 0) {
    *u_throttle = tanhf(mu[0] * bscale);
  } else {
    *u_throttle = tanhf(mu[0] * tscale);
  }
  *u_steering = tanhf(mu[1] * sscale);
}
