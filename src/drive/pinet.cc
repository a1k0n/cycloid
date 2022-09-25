#include <drive/pinet.h>

#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>

using Eigen::VectorXf;
using Eigen::MatrixXf;

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
                       float tscale, float sscale, float *u_throttle, float *u_steering) {
  VectorXf state(6);
  VectorXf linear(HIDDEN);

  // copy embed
  linear = Eigen::Map<VectorXf>(emb[ix], HIDDEN);

  state << v, w, xl, yl, cl, sl;

  // matmul state_weight1 with state
  linear.noalias() += Eigen::Map<MatrixXf>((float*)state_weight1, 6, HIDDEN).transpose() * state +
                      Eigen::Map<VectorXf>((float*)state_bias1, HIDDEN);

  // relu 1
  for (int i = 0; i < HIDDEN; i++) {
    if (linear[i] < 0) {
        linear[i] = 0;
    }
  }

  // linear 2
  linear = Eigen::Map<MatrixXf>((float *)linear_weight1, HIDDEN, HIDDEN).transpose() * linear +
           Eigen::Map<VectorXf>((float *)linear_bias1, HIDDEN);

  // relu 2
  for (int i = 0; i < HIDDEN; i++) {
    if (linear[i] < 0) {
        linear[i] = 0;
    }
  }

  // mu layer
  VectorXf mu = Eigen::Map<MatrixXf>((float *)mu_weight, HIDDEN, 2).transpose() * linear +
                Eigen::Map<VectorXf>((float *)mu_bias, 2);

  *u_throttle = tanhf(mu[0] * tscale);
  *u_steering = tanhf(mu[1] * sscale);
}
