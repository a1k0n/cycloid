import numpy as np
from numpy import sin, cos
import gensystem


def curvature(s):
    if s < 10:
        return 0
    if s < 10 + 5*np.pi/2:  # 90 degree bend
        return 1.0/5
    else:
        return 0


def f(X, U, dt):
    x, y, theta, s, psi, ye = X
    delta, = U
    k = curvature(s)
    v = 1  # 1/(abs(k)+eps)
    w = delta*v

    dxdt = cos(theta + w*0.5*dt) * v
    dydt = sin(theta + w*0.5*dt) * v

    dpsi = w - v*cos(psi)*k / (1 - k*y)
    psinew = psi + 0.5*dt*dpsi
    dy = dt*v*sin(psinew)
    ds = dt*v*cos(psinew) / (1 - k*y)

    V = -ds # + y**2

    return np.array([x+dt*dxdt, y+dt*dydt, theta+dt*w,
                     s+ds, psi+dt*dpsi, ye+dy]), V


def ddpiter(X, U, origV, dt):
    # apply policy forward, adjust V backward
    T = X.shape[0]
    K = np.zeros((T, 2))
    k = np.zeros(T)
    V, Vx, Vxx = 0, np.zeros(2), np.zeros((2, 2))
    damp = 0
    print 'final V, Vx, Vxx (state', X[-1], ')', V, Vx, Vxx[0, 0], Vxx[0, 1], Vxx[1, 1]
    for i in range(T-1, -1, -1):
        k[i], K[i], V, Vx, Vxx = gensystem.ddpbackstep(X[i], U[i], V, Vx, Vxx, dt, damp, curvature)
        print i, 'V, k, K, Vx, Vxx', V, k[i], K[i], Vx, Vxx[0, 0], Vxx[0, 1], Vxx[1, 1]

    alpha = 1.0
    while True:
        Xnew = X.copy()
        Unew = U.copy()
        newV = 0
        for i in range(T-1):
            Unew[i] += alpha * k[i] + np.dot(K[i], Xnew[i, -2:] - X[i, -2:])
            Xnew[i+1], dV = f(Xnew[i], Unew[i], dt)
            newV += dV
        # finalV, _, _ = gensystem.finalcost(Xnew[-1], dt, curvature)
        # newV += finalV
        print 'alpha', alpha, 'newV', newV, 'oldV', origV
        if newV < origV:
            return Xnew, Unew, newV
        if alpha < 2**-16:
            print 'failed to get better solution, alpha', alpha, 'V', origV, 'newV', newV
            return Xnew, Unew, newV
        alpha /= 2.0


def main():
    from matplotlib import pyplot as plt
    T = 30

    Ns = 6
    X = np.zeros((T, Ns))
    U = np.zeros((T, 1))
    x = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    # x = np.zeros(6)
    dt = 1.0 / 30
    # invent an initial trajectory with a naive controller
    V = 0
    for i in range(T):
        X[i] = x
        s = x[3]
        psi = x[4]
        y = x[5]
        k = curvature(s)
        # print i, s, k, x
        U[i] = k - psi - y
        x, dV = f(x, U[i], dt)
        V += dV
    s = x[3]
    print 'original control sequence', U
    print 'final distance', s, 'cost', V
    plt.plot([10], [5], 'x')
    plt.plot(X[:, 0], X[:, 1])
    for i in range(20):
        X, U, V = ddpiter(X, U, V, dt)
        if X is None:
            break
    plt.plot(X[:, 0], X[:, 1])
    print 'final distance', X[-1, 3], '/', s

    print 'final control sequence', U

    plt.axis('equal')
    plt.show()


def testV():
    from matplotlib import pyplot as plt
    Vs = []
    t = np.linspace(-20, 20, 100)
    x = np.array([0.0, 1.0, 0.1, 0.0, 0.1, 1.0])
    for d in t:
        x1, V = f(x, [d], 1.0/30.0)
        x2, dV = f(x1, [d], 1.0/30.0)
        V += dV
        x3, dV = f(x2, [d], 1.0/30.0)
        V += dV
        Vs.append(V)

    u = [0]
    x1, _ = f(x, u, 1.0/30)
    x2, _ = f(x1, u, 1.0/30)
    #x3, _ = f(x2, u, 1.0/30)
    V, Vx, Vxx = 0, np.zeros((1, 2)), np.zeros((2, 2))
    d = 1e-9  # damping
    #print 'Vx', Vx, 'Vxx', Vxx.reshape(-1)
    #_, _, V, Vx, Vxx = gensystem.ddpbackstep(x3, u, V, Vx, Vxx, 1.0/30, d, curvature)
    #print 'Vx', Vx, 'Vxx', Vxx.reshape(-1)
    #_, _, V, Vx, Vxx = gensystem.ddpbackstep(x2, u, V, Vx, Vxx, 1.0/30, d, curvature)
    #print 'Vx', Vx, 'Vxx', Vxx.reshape(-1)
    _, _, V, Vx, Vxx = gensystem.ddpbackstep(x1, u, V, Vx, Vxx, 1.0/30, d, curvature)
    print 'Vx', Vx, 'Vxx', Vxx.reshape(-1)
    _, _, V, Vx, Vxx = gensystem.ddpbackstep(x, u, V, Vx, Vxx, 1.0/30, d, curvature)
    print 'Vx', Vx, 'Vxx', Vxx.reshape(-1)
    Qx, Qu, Qxx, Qux, Quu = gensystem.Q(x, u, Vx, Vxx, 1.0/30, d, curvature)
    print 'Qu', Qu
    print 'Quu', Quu
    V = Vs[50]

    # [1 dx.T du.T] * [Qx.T*dx + Qu.T*du]
    #             [Qx + Qxx*dx + Qux.T*du]
    #             [Qu + Qux*dx + Quu*du]
    # Qx.T*dx + Qu.T*du + dx.T*Qx + dx.T*Qxx*dx +
    # + du.T*Qu*du + 2du.T*Qux*dx + du.T*Quu*du
    # dx = 0, just evaluate du
    # Qu*du + du(Qu.T + Quu*du)

    plt.plot(t, Vs)
    i = np.argmin(Vs)
    plt.plot(t[i], Vs[i], 'o')
    plt.show()

    plt.plot(t, Vs)
    plt.plot(t[i], Vs[i], 'o')
    dQ = (Qu*t + 0.5*t*Quu*t)[0] + V
    i = np.argmin(dQ)
    plt.plot(t, dQ)
    plt.plot(t[i], dQ[i], 'o')
    #linQ = V + t*Qu[0]
    #plt.plot(t, linQ)
    plt.show()


if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    testV()
    # main()
