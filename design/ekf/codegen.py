import sympy as sp
from sympy.printing.python import PythonPrinter

C_UFs = {
    "Heaviside": "Heaviside",
    "DiracDelta": "DiracDelta",
    "Abs": "fabsf",
    "Min": "fminf",
    "Max": "fmaxf"
}

pyprint = PythonPrinter(None)


def ccode(expr):
    return sp.ccode(expr, user_functions=C_UFs)


def pycode(expr):
    return pyprint._str(expr)


def ccode_matrix(matexpr, indent):
    lines = []
    H, W = matexpr.shape
    indent = ' '*indent
    for j in range(H):
        lines.append(', '.join([ccode(expr) for expr in matexpr[W*j:W*(j+1)]]))
    return (',\n' + indent).join(lines) + ';'


def pycode_matrix(matexpr, indent):
    lines = []
    H, W = matexpr.shape
    indent = ' '*indent
    for j in range(H):
        lines.append('[' + ', '.join(
            [pycode(expr) for expr in matexpr[W*j:W*(j+1)]]) + ']')
    return (',\n' + indent).join(lines)


class EKFGen:
    def __init__(self, X):
        self.X = X
        self.N = len(X)
        self.fh = None
        self.fcc = None
        self.fpy = None

    def open(self, x0, P0):
        # FIXME: make class name, file name, path flexible
        self.fh = open("ekf.h", "w")
        self.fcc = open("ekf.cc", "w")
        self.fpy = open("ekf.py", "w")
        N = self.N

        print >>self.fh, '''#ifndef MODEL_EKF_H_
#define MODEL_EKF_H_
#include <Eigen/Dense>

class EKF {
 public:
  EKF();
  
  void Reset();
'''
        print >>self.fcc, '''#include <Eigen/Dense>
#include "ekf.h"

using Eigen::VectorXf;
using Eigen::MatrixXf;

#define Min(x, y) fminf(x, y)

static inline float Heaviside(float x) {
  return x < 0 ? 0 : x;
}

static inline float DiracDelta(float x) {
  return x == 0;
}

EKF::EKF() : x_(%d), P_(%d, %d) {
  Reset();
}

''' % (N, N, N)
        print >>self.fcc, 'void EKF::Reset() {'
        print >>self.fcc, '  x_ <<', ccode_matrix(x0, 8)
        print >>self.fcc, '  P_.setIdentity();'
        print >>self.fcc, '  P_.diagonal() <<', ccode_matrix(P0, 4)
        print >>self.fcc, '}\n'

        print >>self.fpy, '''#!/usr/bin/env python
import numpy as np

def Heaviside(x):
    return np.maximum(0, x)


def initial_state():'''
        print >>self.fpy, '    x = np.float32(['
        print >>self.fpy, '        ' + pycode_matrix(x0, 8)
        print >>self.fpy, '    ])'
        print >>self.fpy, '\n    return x, P\n\n'

    def close(self):
        print >>self.fh, '''

  const Eigen::VectorXf& GetState() { return x_; }
  const Eigen::MatrixXf& GetCovariance() { return P_; }

 private:
  Eigen::VectorXf x_;
  Eigen::MatrixXf P_;
};

#endif  // MODEL_EKF_H_
'''
        self.fh.close()
        self.fcc.close()
        self.fpy.close()

    def generate_predict(self, f, u, Q, dt):
        ''' Generate a method for doing an EKF prediction step. f(X, u) -> X is
        the system dynamics model, X is a symbolic vector of each state
        variable name u is the symbolic control input vector. '''
        N = self.N
        F = f.jacobian(self.X)
        vs, es = sp.cse([F - sp.eye(N), f - self.X, Q], optimizations='basic',
                        symbols=sp.numbered_symbols("tmp"))

        self.generate_predict_cc(f, u, Q, dt, N, F, vs, es)
        self.generate_predict_py(f, u, Q, dt, N, F, vs, es)

    def generate_predict_py(self, f, u, Q, dt, N, F, vs, es):
        # maybe we should just lambdify this or ufuncify
        arglist = ['x', 'P', pycode(dt)] + [pycode(ui) for ui in u]
        print >>self.fpy, "def predict(%s):" % ', '.join(arglist)

        print >>self.fpy, "    (%s) = x" % ', '.join([pycode(x) for x in self.X])
        print >>self.fpy, ""

        for x in vs:
            print >>self.fpy, '    %s = %s' % (pycode(x[0]), pycode(x[1]))

        print >>self.fpy, '\n    F = np.eye(%d)' % N
        for i, term in enumerate(es[0]):
            if term != 0:
                print >>self.fpy, '    F[%d, %d] += %s' % (
                    i / N, i % N, pycode(term))

        print >>self.fpy, '    Q = np.float32([', ', '.join(
            [ccode(x*x) for x in es[2]]) + '])'

        for i, term in enumerate(es[1]):
            if term != 0:
                print >>self.fpy, '    x[%d] += %s;' % (i, pycode(term))

        print >>self.fpy, '\n    P = np.dot(Fk_1, np.dot(P, Fk_1.T)) + Delta_t * np.diag(Q)'

        print >>self.fpy, "    return x, P\n\n"

    def generate_predict_cc(self, f, u, Q, dt, N, F, vs, es):
        N = self.N

        arglist = ["float " + ccode(dt)] + ["float " + ccode(ui) for ui in u]
        print >>self.fcc, "void EKF::Predict(%s) {" % ', '.join(arglist)
        print >>self.fh, '  void Predict(%s);' % ', '.join(arglist)

        for i, elem in enumerate(self.X):
            if elem in (f - self.X).free_symbols:
                print >>self.fcc, "  float %s = x_[%d];" % (ccode(elem), i)

        print >>self.fcc, ""

        for x in vs:
            print >>self.fcc, '  float %s = %s;' % (ccode(x[0]), ccode(x[1]))

        print >>self.fcc, '\n  MatrixXf F(%d, %d);' % (N, N)
        print >>self.fcc, '  F.setIdentity();'
        for i, term in enumerate(es[0]):
            if term != 0:
                print >>self.fcc, '  F(%d, %d) += %s;' % (i / N, i % N, ccode(term))

        print >>self.fcc, '\n  VectorXf Q(%d);' % N
        print >>self.fcc, '  Q <<', ', '.join([ccode(x*x) for x in es[2]]) + ';'

        for i, term in enumerate(es[1]):
            if term != 0:
                print >>self.fcc, '  x_[%d] += %s;' % (i, ccode(term))
        
        print >>self.fcc, '\n  P_ = F * P_ * F.transpose();'
        print >>self.fcc, '  P_.diagonal() += ' + ccode(dt) + ' * Q;'
        print >>self.fcc, '}\n'


    def generate_measurement(self, name, h_x, h_z, z_k, R_k):
        H = h_x.jacobian(self.X)
        M = h_z.jacobian(z_k)
        y_k = h_z - h_x
        vs, es = sp.cse([y_k, H, M], optimizations='basic',
                        symbols=sp.numbered_symbols("tmp"))

        self.generate_measurement_cc(
            name, h_x, h_z, z_k, R_k, H, M, y_k, vs, es)

        self.generate_measurement_py(
            name, h_x, h_z, z_k, R_k, H, M, y_k, vs, es)

    def generate_measurement_cc(self, name, h_x, h_z, z_k, R_k,
                                H, M, y_k, vs, es):
        ''' Generate a method for doing an EKF measurement update. h(X) -> X is the
        measurement function of the state, X is the symbolic state vector, z_k is
        the symbolic measurement vector, R is the measurement noise covariance
        in terms of z_k (either full matrix or vector; if a vector, treated as std.
        deviations along the diagonal)

        h_x is the measurement vector in terms of state variables
        h_z is the same vector but in terms of the measurement (possibly identical)
        '''

        N = self.N
        arglist = ["float " + ccode(ui) for ui in z_k]
        if not R_k.is_Matrix and R_k.is_Symbol:
            arglist.append("Eigen::MatrixXf Rk")
        name = name[0].upper() + name[1:]
        print >>self.fcc, 'bool EKF::Update%s(%s) {' % (name, ', '.join(arglist))
        print >>self.fh, '  bool Update%s(%s);' % (name, ', '.join(arglist))

        for i, elem in enumerate(self.X):
            if elem in h_x.free_symbols:
                print >>self.fcc, "  float %s = x_[%d];" % (ccode(elem), i)

        for x in vs:
            print >>self.fcc, '  float %s = %s;' % (ccode(x[0]), ccode(x[1]))

        print >>self.fcc, ""

        print >>self.fcc, '\n  VectorXf yk(%d);' % (y_k.shape[0])
        print >>self.fcc, '  yk <<', ccode_matrix(es[0], 8)

        print >>self.fcc, '\n  MatrixXf Hk(%d, %d);' % H.shape
        print >>self.fcc, '  Hk <<', ccode_matrix(es[1], 8)

        if R_k.is_Matrix:
            if R_k.shape[1] > 1:
                print >>self.fcc, '\n  MatrixXf Rk(%d, %d);' % R_k.shape
                print >>self.fcc, '  Rk <<', ccode_matrix(R_k, 8)
            else:
                print >>self.fcc, '\n  VectorXf Rk(%d);' % R_k.shape[0]
                # in this case we need to square it also
                # print >>self.fcc, '  Rk <<', ccode_matrix(R_k, 8)
                print >>self.fcc, '  Rk <<', ', '.join([ccode(x*x) for x in R_k]) + ';'

        if not M.is_Identity:
            print >>self.fcc, '  MatrixXf Mk(%d, %d);' % M.shape
            print >>self.fcc, '  Mk <<', ccode_matrix(es[2], 8)
            print >>self.fcc, '  Rk = Mk * Rk * Mk.transpose();'

        if R_k.is_Matrix and R_k.shape[1] == 1:
            print >>self.fcc, '\n  MatrixXf S = Hk * P_ * Hk.transpose();'
            print >>self.fcc, '  S.diagonal() += Rk;'
        else:
            print >>self.fcc, '\n  MatrixXf S = Hk * P_ * Hk.transpose() + Rk;'
        print >>self.fcc, '  MatrixXf K = P_ * Hk.transpose() * S.inverse();'
        # FIXME: return false if S is not invertible?

        print >>self.fcc, '\n  x_.noalias() += K * yk;'
        print >>self.fcc, '  P_ = (MatrixXf::Identity(%d, %d) - K*Hk) * P_;' % (N, N)

        print >>self.fcc, '  return true;'
        print >>self.fcc, '}\n'

    def generate_measurement_py(self, name, h_x, h_z, z_k, R_k,
                                H, M, y_k, vs, es):
        arglist = ['x', 'P'] + [str(u_i) for u_i in z_k]
        if not R_k.is_Matrix and R_k.is_Symbol:
            arglist = arglist + ["Rk"]
        print >>self.fpy, 'def update_%s(%s):' % (name, ', '.join(arglist))

        for i, elem in enumerate(self.X):
            if elem in h_x.free_symbols:
                print >>self.fpy, "    %s = x[%d]" % (pycode(elem), i)

        for x in vs:
            print >>self.fpy, '    %s = %s' % (pycode(x[0]), pycode(x[1]))

        print >>self.fpy, '\n    yk = np.float32(['
        print >>self.fpy, ' '*8 + pycode_matrix(es[0], 8) + '])'

        print >>self.fpy, '\n    Hk = np.float32(['
        print >>self.fpy, ' '*8 + pycode_matrix(es[1], 8) + '])'

        if R_k.is_Matrix:
            if R_k.shape[1] > 1:
                print >>self.fpy, '\n    Rk = np.float32(['
                print >>self.fpy, ' '*8 + pycode_matrix(R_k, 8) + '])'
            else:
                # in this case we need to square it also
                print >>self.fpy, '\n    Rk = np.diag(['
                print >>self.fpy, ' '*8 + ', '.join(
                    [pycode(x*x) for x in R_k]) + '])'

        if not M.is_Identity:
            print >>self.fpy, '    Mk = np.float32(['
            print >>self.fpy, ' '*8 + pycode_matrix(es[2], 8) + '])'
            print >>self.fpy, '    Rk = np.dot(Mk, np.dot(Rk, Mk.T))'

        print >>self.fpy, '\n    S = np.dot(Hk, np.dot(P, Hk.T)) + Rk'
        # linalg.solve? lstsq?
        print >>self.fpy, '    K = np.dot(P, np.dot(Hk.T, np.linalg.inv(S)))'
        # FIXME: return false if S is not invertible?

        print >>self.fpy, '''    x += np.dot(K, yk)
    KHk = np.dot(K, Hk)
    P = np.dot((np.eye(len(x)) - KHk), P)
    return x, P

'''
