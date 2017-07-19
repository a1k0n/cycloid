import sympy as sp

C_UFs = {"Heaviside": "heaviside", "DiracDelta": "diracdelta", "Abs": "fabsf"}


def ccode(expr):
    return sp.ccode(expr, user_functions=C_UFs)


# TODO: open method which starts python *and* C++ outputs
# make this into a class, I guess, then you can just hold onto X

def ccode_matrix(matexpr, indent):
    lines = []
    H, W = matexpr.shape
    indent = ' '*indent
    for j in range(H):
        lines.append(', '.join([ccode(expr) for expr in matexpr[W*j:W*(j+1)]]))
    return (',\n' + indent).join(lines) + ';'


def generate_predict_cc(f, X, u, Q, dt):
    ''' Generate a method for doing an EKF prediction step. f(X, u) -> X is the
    system dynamics model, X is a symbolic vector of each state variable name u
    is the symbolic control input vector. '''

    F = f.jacobian(X)

    arglist = ["float " + ccode(dt)] + ["float " + ccode(ui) for ui in u]
    print "void EKF::Predict(%s) {" % ", ".join(arglist)

    for i, elem in enumerate(X):
        print "  float %s = x_[%d];" % (ccode(elem), i)

    print ""

    vs, es = sp.cse([F - sp.eye(len(X)), f - X, Q], optimizations='basic',
                    symbols=sp.numbered_symbols("tmp"))
    for x in vs:
        print '  float %s = %s;' % (ccode(x[0]), ccode(x[1]))

    N = len(X)
    print '\n  MatrixXf F(%d, %d);'
    print '  F.setIdentity();'
    for i, term in enumerate(es[0]):
        if term != 0:
            print '  F(%d, %d) += %s;' % (i / N, i % N, ccode(term))

    print '\n  VectorXf Q(%d);' % Q.shape[0]
    print '  Q <<', ', '.join([ccode(x*x) for x in es[2]]) + ';'

    for i, term in enumerate(es[1]):
        if term != 0:
            print '  x_[%d] += %s;' % (i, ccode(term))
    
    print '\n  P_ = Fk * P_ * Fk.transpose();'
    print '  P_.diagonal() += ' + ccode(dt) + ' * Q;'
    print '}'


def generate_measurement_cc(name, h_x, h_z, X, z_k, R_k):
    ''' Generate a method for doing an EKF measurement update. h(X) -> X is the
    measurement function of the state, X is the symbolic state vector, z_k is
    the symbolic measurement vector, R is the measurement noise covariance
    in terms of z_k (either full matrix or vector; if a vector, treated as std.
    deviations along the diagonal)

    h_x is the measurement vector in terms of state variables
    h_z is the same vector but in terms of the measurement (possibly identical)
    '''

    H = h_x.jacobian(X)
    M = h_z.jacobian(z_k)

    y_k = h_z - h_x
    vs, es = sp.cse([y_k, H, M], optimizations='basic',
                    symbols=sp.numbered_symbols("tmp"))

    arglist = ["float " + ccode(ui) for ui in z_k]
    if not R_k.is_Matrix and R_k.is_Symbol:
        arglist.append("MatrixXf Rk")
    name = name[0].upper() + name[1:]
    print 'EKF::Update%s(%s) {' % (name, ', '.join(arglist))

    for i, elem in enumerate(X):
        # only if used?
        print "  float %s = x_[%d];" % (ccode(elem), i)

    for x in vs:
        print '  float %s = %s;' % (ccode(x[0]), ccode(x[1]))

    print ""

    print '\n  VectorXf yk(%d);' % (y_k.shape[0])
    print '  yk <<', ccode_matrix(es[0], 8)

    print '\n  MatrixXf Hk(%d, %d);' % H.shape
    print '  Hk <<', ccode_matrix(es[1], 8)

    if R_k.is_Matrix:
        if R_k.shape[1] > 1:
            print '\n  MatrixXf Rk(%d, %d);' % R_k.shape
            print '  Rk <<', ccode_matrix(R_k, 8)
        else:
            print '\n  Vector3f Rk(%d);' % R_k.shape[0]
            # in this case we need to square it also
            # print '  Rk <<', ccode_matrix(R_k, 8)
            print '  Rk <<', ', '.join([ccode(x*x) for x in R_k]) + ';'

    if not M.is_Identity:
        print '  {'
        print '    MatrixXf M(%d, %d);' % M.shape
        print '    M <<', ccode_matrix(es[2], 9)
        print '    Rk = Mk * Rk * Mk.transpose();'
        print '  }'

    if R_k.is_Matrix and R_k.shape[1] == 1:
        print '\n  Matrix3f S = Hk * P_ * Hk.transpose();'
        print '  S.diagonal() += Rk;'
    else:
        print '\n  Matrix3f S = Hk * P_ * Hk.transpose() + Rk;'
    print '  MatrixXf K = P_ * Hk.transpose() * S.inverse();'

    print '\n  x_.noalias() += K * yk;'
    print '  P_ = (MatrixXf::Identity(%d, %d) - K*Hk) * P_;' % (
        X.shape[0], X.shape[0])

    print '}\n'
