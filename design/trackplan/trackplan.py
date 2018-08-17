# racetrack route planner
# based on apex cone locations and track widths, get a 
import numpy as np


maxv = 10
laterala = 8
maxk = 1.5
bw_v = np.pi*2*0.7
bw_w = np.pi*2*1.5

# track is a set of points and radii (positive or negative if track goes CCW/CW
# around each point)
# so first we determine the nearest point

# T is [5, NUM_PTS]; [(x, y, r), i]
# TODO: precompute handy normals


def gettargetv(k):
    kmin = laterala / (maxv**2)
    targetv = maxv
    if np.abs(k) > kmin:
        targetv = np.sqrt(laterala / np.abs(k))
    return targetv


# get nearest point on track, its direction normal and curvature
def gettrack(xy, T):
    Tn = np.hstack([T[:, 1:], T[:, :1]])
    Nn = Tn[:2] - T[:2]
    L = np.linalg.norm(Nn, axis=0)
    S = (Tn[2] - T[2]) / L
    C = np.sqrt(1 - S**2)
    Nn /= L
    Nn = np.vstack([-Nn[0]*S - Nn[1]*C, Nn[0]*C - Nn[1]*S])

    # we need to rotate Nn / Np based on the radii of the current/next/previous points
    # ...unless they have the same radius, in which case this doesn't work.
    # ln = np.linalg.norm(Tn[:2] - T[:2], axis=0) * T[2] / (Tn[2] - T[2])
    # print 'ln', ln
    # crap. for now let's just ignore the problem

    Pn = (Tn[:2] + Nn*Tn[2]).T
    P = (T[:2] + Nn*T[2]).T

    tnum = np.sum((Pn - P)*(xy - P), axis=1)
    tden = np.sum((Pn - P)**2, axis=1)
    t = np.clip(tnum / tden, 0, 1)
    # closest point on each edge in the polygon to xy
    pxy = (P.T*(1-t) + Pn.T*t).T
    dists = np.sqrt(np.sum((pxy-xy)**2, axis=1))
    i = np.argmin(dists)
    if t[i] == 0 or t[i] == 1:
        if t[i] == 1:
            i = (i+1) % T.shape[1]
        # closest point is one of the circles
        dp = xy - T[:2, i].T
        dp /= np.linalg.norm(dp)
        p = T[:2, i].T + dp * np.abs(T[2, i])
        n = np.array([dp[1], -dp[0]]) * np.sign(T[2, i])
        return p, n, 1.0/T[2, i], gettargetv(1.0/T[2, i])
    else:
        # closest point is on the linear sections
        n = Pn[i] - P[i]
        n /= np.linalg.norm(n)
        finalv = gettargetv(1.0/Tn[2, i])
        # need to compute deceleration
        tt = t[i]**2
        return pxy[i], n, 0, maxv*(1-tt) + tt*finalv

    return None


def step(X, u, targetv, dt):
    # X = [x y theta v w]
    # velocity control
    print 'u', u, X[4], 'v', targetv, X[3]
    if targetv > X[3]:
        ebw = np.exp(-bw_v * dt)
    else:
        ebw = np.exp(-bw_v * 2 * dt)
    vnew = (1 - ebw) * targetv + ebw * X[3]
    v = (X[3] + vnew) * 0.5

    # yaw rate control
    targetw = v * u
    ebw = np.exp(-bw_w * dt)
    wnew = (1 - ebw) * targetw + ebw * X[4]
    thetanew = X[2] + wnew * dt
    theta = X[2] + wnew * dt * 0.5

    X[0] += np.cos(theta)*v*dt
    X[1] += np.sin(theta)*v*dt
    X[2] = thetanew
    X[3] = vnew
    X[4] = wnew

    return X


def drive(X, dt):
    p, n, k, v = gettrack(X[:2], T)
    nx = np.array([n[1], -n[0]])
    ye = np.dot(X[:2] - p, nx)
    C, S = np.cos(X[2]), np.sin(X[2])
    R = np.array([[C, S], [-S, C]])
    Rn = np.dot(R, n)
    # not sure if psie is backwards or not
    psie = np.arctan2(Rn[1], Rn[0])
    # print n, C, S, ye, psie, k
    Cp = np.cos(psie)
    Sp = np.sin(psie)
    Cpy = Cp / (1 - k * ye)
    Kpy = 1.0
    Kvy = 5.0
    ds = X[3]*Cpy*dt
    return -Cpy*(ye*Cpy*(-Kpy*Cp) + Sp*(k*Sp - Kvy*Cp) + k), v, ds


def trackexport(T):
    ''' compute various positions and normals for export '''
    output = np.zeros((9, T.shape[1]))

    output[:3] = T[:3]  # first three dimensions of output are unchanged

    Tn = np.hstack([T[:, 1:], T[:, :1]])
    Nn = Tn[:2] - T[:2]
    L = np.linalg.norm(Nn, axis=0)
    S = (Tn[2] - T[2]) / L
    C = np.sqrt(1 - S**2)
    Nn /= L
    Nn = np.vstack([-Nn[0]*S - Nn[1]*C, Nn[0]*C - Nn[1]*S])
    Nn /= np.linalg.norm(Nn, axis=0)

    # we need to rotate Nn / Np based on the radii of the current/next/previous points
    # ...unless they have the same radius, in which case this doesn't work.
    # ln = np.linalg.norm(Tn[:2] - T[:2], axis=0) * T[2] / (Tn[2] - T[2])
    # print 'ln', ln
    # crap. for now let's just ignore the problem

    Pn = (Tn[:2] + Nn*Tn[2])
    P = (T[:2] + Nn*T[2])
    output[3:5] = P
    output[5:7] = Pn
    output[7:9] = Nn

    return output


if __name__ == '__main__':
    from matplotlib import pyplot as plt
    T = np.array([
        [0, 0, 1],
        [9, -1, 2],
        [10, -4, 1],
        [5, -3, -1],
        [0, -5, 1],
    ], np.float32).T

    print trackexport(T)

    if False:
        plt.plot(T[0], T[1], 'o')
        t = np.linspace(0, 2*np.pi, 100)
        for x in range(T.shape[1]):
            plt.plot(np.cos(t)*T[2, x] + T[0, x], np.sin(t)*T[2, x] + T[1, x])
        plt.axis('equal')

        xy = np.array([7.0, -3.0])
        pp, n, k, _ = gettrack(xy, T)
        plt.plot(xy[0], xy[1], 'o')

        plt.plot([pp[0], pp[0]+n[0]], [pp[1], pp[1] + n[1]], '-x')

        plt.show()

    if False:
        X = np.zeros(5)
        v = np.zeros(100)
        w = np.zeros(100)
        for i in range(100):
            X = step(X, 2, 5, 1.0/30)
            v[i] = X[3]
            w[i] = X[4]
        plt.plot(v)
        plt.plot(w)
        plt.plot(w/v)
        plt.show()

    if True:
        totalS = 0
        X = np.array([1, 1, 0, 0, 0], np.float32)
        Nsteps = 222*5
        xy = np.zeros((2, Nsteps))
        dt = 1.0 / 30
        y = 0
        for i in range(Nsteps):
            u, v, ds = drive(X, dt)
            u = np.clip(u, -maxk, maxk)
            X = step(X, u, v, dt)
            xy[:, i] = X[:2]
            totalS += ds

        print 'distance around track', totalS
        plt.plot(T[0], T[1], 'o')
        t = np.linspace(0, 2*np.pi, 100)
        for x in range(T.shape[1]):
            plt.plot(np.cos(t)*T[2, x] + T[0, x], np.sin(t)*T[2, x] + T[1, x])
        plt.axis('equal')

        plt.plot(xy[0], xy[1])
        plt.plot(xy[0, -1], xy[1, -1], 'x')
        plt.show()
