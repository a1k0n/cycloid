''' Convert a racetrack .SVG into a perceptual curvature map for localization,
and also compute an optimal racing line given forward and lateral acceleration
limits. '''

import autograd
import autograd.numpy as np
import svgpathtools
import scipy.optimize


TRACK_SPACING = 20  # 20 cm track spacing


def SVGPathToTrackPoints(fname, spacing=TRACK_SPACING):
    path = svgpathtools.parse_path(open(fname).read())

# input: svg based on a 100px / m image
    def next_t(path, t, dist):
        p = path.point(t)
        L = path.length()
        # t += 1.0 / np.abs(path.derivative(t))
        itr = 0
        while itr < 20:
            itr += 1
            p1 = path.point(t)
            err = np.abs(p1 - p) - dist
            d1 = path.derivative(t)
            if np.abs(err) < 1e-5:
                return t, p1, d1 / np.abs(d1)
            derr = np.abs(d1) * L
            # do a step in Newton's method (clipped because some of the
            # gradients in the curve are really small)
            t -= np.clip(err / derr, -1e-2, 1e-2)
            t = np.clip(t, 0, 1)
        return t, p, d1 / np.abs(d1)

    d0 = path.derivative(0)
    pts = [[path.point(0), d0 / np.abs(d0)]]
    t = 0
    while t < 1:
        t, p, d = next_t(path, t, spacing)
        pts.append([p, d])

    return pts


def QuadFitCurvatureMap(x):
    curv = []

    for i in range(len(x)):
        # do a look-ahead quadratic fit, just like the car would do
        pts = x[(np.arange(6) + i) % len(x)] / 100  # convert to meters
        basis = (pts[1] - pts[0]) / np.abs(pts[1] - pts[0])

        # project onto forward direction
        pts = (np.conj(basis) * (pts - pts[0]))

        p = np.polyfit(np.real(pts), np.imag(pts), 2)
        curv.append(p[0] / 2)

    return np.float32(curv)


def TrackCurvature(x):
    # use quadratic b-splines at each point to estimate curvature
    # i get almost the same formula i had before but it's off by a factor of 4!

    xx = np.concatenate([x[-1:], x, x[:1]])
    p0 = xx[:-2]
    p1 = xx[1:-1]
    p2 = xx[2:]
    T = p2 - p0  # track derivative
    uT = np.abs(T)
    TT = 4*(p0 - 2*p1 + p2)
    k = (np.real(T)*np.imag(TT) - np.imag(T)*np.real(TT)) / (uT**3)
    return k


def TrackNormal(x):
    xx = np.concatenate([x[-1:], x, x[:1]])
    p0 = xx[:-2]
    p2 = xx[2:]
    T = p2 - p0  # track derivative
    uT = np.abs(T)
    return T / uT


def TrackSmoothness(x):
    k = TrackCurvature(x)
    kdiff = np.concatenate([k[1:] - k[:-1], k[-1:] - k[:1]])
    return np.sum(kdiff**2)


def SmoothTrack(x0, k1=10):
    N = len(x0)
    B = 1j * TrackNormal(x0)

    def objective(y):
        x = x0 + y * B
        # try to keep points equidistant
        return TrackSmoothness(x) + k1 * np.sum(np.abs((x - x0)**2))

    dobj = autograd.grad(objective)
    yopt = scipy.optimize.fmin_bfgs(
        objective, np.zeros(N), dobj, disp=True)
    return x0 + yopt * B


def ye_limit(x, trackwidth):
    k = TrackCurvature(x)
    N = len(k)
    if type(trackwidth) == tuple:
        lowlimit = trackwidth[0] * np.ones(N)
        highlimit = trackwidth[1] * np.ones(N)
    else:
        lowlimit = -trackwidth/2 * np.ones(N)
        highlimit = trackwidth/2 * np.ones(N)
        # use a 5% margin so we can't actually hit the center of curvature
        lowlimit[k < 0] = np.maximum(0.95/k[k < 0], -trackwidth/2)
        highlimit[k > 0] = np.minimum(0.95/k[k > 0], trackwidth/2)
    return lowlimit, highlimit


def TrackVelocity(x, k, vmax, acmax, Ta):
    ''' compute the velocity at each point along the track (given
    already-computed curvatures) assuming a certain accelration profile '''
    v = np.minimum(np.abs(acmax / k)**0.5, vmax)

    # also compute arc distance between successive points in x given curvature
    # k; for now we'll just use the linear distance though as it's close enough
    s = np.abs(np.concatenate([x[1:] - x[:-1], x[:1] - x[-1:]]))

    T = 0
    vout = []

    # first pass is just to get the initial velocity
    # let's assume it's zero
    # for i in range(1, len(k)):
    #     va = va + (v[i] - va) / Ta

    # TODO: braking backward pass
    va = vmax
    for i in range(len(k)-1, -1, -1):
        dt = s[i] / va  # time to reach next waypoint
        # (v' + speed_const*dt/Tv) / (1 - dt/Tv) = v
        va = min(vmax, (va + 0 * dt/Ta) / (1 - dt/Ta))
        va = min(va, v[i])
        v[i] = va

    va = 0
    for i in range(0, len(k)):
        a = (v[i] - va) / Ta  # acceleration
        dt = s[i] / (va + a/2)  # time to reach next waypoint
        va = np.minimum(va + dt*a, v[i])
        T += dt
        vout.append(va)
    return np.array(vout), T


def OptimizeTrack(x, lanewidth=1.8, kcurv=0.2, kdist=0.5):
    ''' returns a list of track offsets which result in a smooth, fast path
    lanewidth is the lane width around centerline in meters
    kcurv controls the tradeoff between maximizing smoothness and minimizing
    curvature
    '''

    u = 1j * TrackNormal(x)
    kmin = 0.09

    def ye_cost(ye):
        rx = u*ye + x
        k = TrackCurvature(rx)
        kmag = np.maximum(np.abs(k), kmin)
        # v = 1e-3 / np.maximum(np.abs(k), kmin)
        curvediff = np.sum((k[1:] - k[:-1])**2) + (k[0] - k[-1])**2
        # v = np.minimum(np.abs(acmax / k)**0.5, vmax)
        dist = np.concatenate([np.abs(rx[1:] - rx[:-1]),
                               np.abs(rx[:1] - rx[-1:])])
        # T = dist/v
        return kcurv*np.sum(kmag) + curvediff + kdist*np.sum(dist)

    dye = autograd.grad(ye_cost)
    bounds = zip(*ye_limit(x, lanewidth))

    return scipy.optimize.fmin_l_bfgs_b(
        ye_cost, np.zeros(len(x)), lambda x: np.array(dye(x)), bounds=bounds, disp=True)


def OptimizeTrack2(x, ye, lanewidth=0.8, vmax=10, acmax=9, Ta=3.):
    ''' returns a list of track offsets which result in a fast path
    within the specified velocity, traction, and acceleration limits:
    lanewidth is the lane width around centerline in meters
    vmax is the maximum velocity in m/s
    acmax is the maximum centrifugal acceleration in a turn
    Ta is the time constant of acceleration, scaled by some weird factor
    '''

    u = 1j * TrackNormal(x)

    def ye_cost(ye):
        rx = u*ye + x
        k = TrackCurvature(rx)

        _, T = TrackVelocity(rx, k, vmax, acmax, Ta)
        return T

    dye = autograd.grad(ye_cost)
    bounds = zip(*ye_limit(x, lanewidth))

    return scipy.optimize.fmin_l_bfgs_b(
        ye_cost, ye, dye, bounds=bounds, disp=True)


def RelativePsie(ye, xm):
    Nx = TrackNormal(xm)
    u = 1j * Nx
    rx = u*ye + xm
    Nr = TrackNormal(rx)

    # psie is sort of backwards: higher angles go to the left
    return np.angle(Nx / Nr)  # - np.angle(Nr)


if __name__ == '__main__':
    import sys

    TRACK_SPACING = 10  # cm
    x = SVGPathToTrackPoints(sys.argv[1], TRACK_SPACING)[:-1]
    x = x[::-1]  # path is backwards?

    xm = np.array(x)[:, 0] / 50  # 50 pixels / meter
    track_k = TrackCurvature(xm)
    Nx = TrackNormal(xm)
    u = 1j * Nx
    np.savetxt("track_x.txt",
               np.vstack([np.real(xm), np.imag(xm)]).T.reshape(-1),
               newline=",\n")
    np.savetxt("track_u.txt",
               np.vstack([np.real(u), np.imag(u)]).T.reshape(-1),
               newline=",\n")
    np.savetxt("track_k.txt", track_k, newline=",\n")

    ye, val, stuff = OptimizeTrack(xm, 1.0, 0.05, 0.4)
    psie = RelativePsie(ye, xm)

    rx = u*ye + xm
    raceline_k = TrackCurvature(rx)

    np.savetxt("raceline_k.txt", raceline_k, newline=",\n")
    np.savetxt("raceline_ye.txt", ye, newline=",\n")
    np.savetxt("raceline_psie.txt", psie, newline=",\n")
    np.savetxt("raceline_ds.txt", np.abs(  # distance between successive points
        np.concatenate([rx[1:] - rx[:-1], rx[:1] - rx[-1:]])), newline=",\n")

    print "saved track_k.txt"
    print "saved raceline_{k, ye, psie}.txt"
