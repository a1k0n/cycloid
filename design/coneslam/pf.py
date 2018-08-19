# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import pickle
import cv2

import annotate
import recordreader
import coneclassify


VIDEO = False
np.set_printoptions(suppress=True)

# relative landmark locations, with unknown scale -- pixel measurements
# from a google maps screenshot
L0 = np.array([
    [0, 0],
    [0, 296],
    [0, 482],
    [0, 778],
    [716, 0],
    [716, 778],
    [425, 296],
    [425, 482],
])
XOFF = 123
YOFF = 475

L0[:, 1] -= 389
# 482-296 units here is exactly 3.6m
# wheel encoder ticks are 2cm apart (wheel is 40cm in diameter, and there are
# 20 ticks / revolution)
# 3.6m should be 180 ticks, and we have 186 pixels
# (so a pixel is pretty close to 2cm^2)
a = 180.0/186.0  # fudge factor for scale w.r.t. ticks
L = L0*a

L = np.array([
    [-76.2, 0],
    [76.2, 0],
    [76.2, -104.14],
    [-137.16, -104.14],
#    [-76.2, -104.14],
])

# oakland track
L = np.array([
    [141, -125],
    [653, -185],
    [607, -336],
    [377, -265],
    [123, -338],
])
L[:, 0] -= 408
L[:, 1] += 102
a = 1

# 408, 102

XOFF = 408
YOFF = 102

NOISE_ANGULAR = 0.008
NOISE_LONG = 16
NOISE_LAT = 8
LM_SELECTIVITY = 50  # 80


def step(X, dt, encoder_dx, gyro_dtheta):
    N = X.shape[1]
    theta0 = X[2]
    theta1 = theta0 + gyro_dtheta*dt + np.random.randn(N) * NOISE_ANGULAR * encoder_dx * dt

    S = np.sin((theta0 + theta1) * 0.5)
    C = np.cos((theta0 + theta1) * 0.5)

    dx = encoder_dx + np.random.randn(N) * NOISE_LONG * encoder_dx * dt
    dy = np.random.randn(N) * NOISE_LAT * encoder_dx * dt

    X[0] += dx*C - dy*S
    X[1] += dx*S + dy*C
    X[2] = theta1


def likeliest_lm(X, L, l):
    # X is [Np, 3]
    # L is [Nl, 2]
    # we need to check Np x Nl combinations

    # dxy[l, :, p] is the relative position between particle p and landmark l
    dxy = L[:, :, None] - X[:2]
    S, C = np.sin(X[2]), np.cos(X[2])
    # rotate each landmark into each particle's frame (y, z)
    z = dxy[:, 0]*C + dxy[:, 1]*S
    y = dxy[:, 0]*S - dxy[:, 1]*C
    # get the relative angle
    LL = -LM_SELECTIVITY*(np.arctan2(y, z) - l)**2

    # normalize probabilities just for tuning
    LL -= np.max(LL)
    LL -= np.log(np.sum(np.exp(LL)))

    j = np.argmax(LL, axis=0)
    return j, np.max(LL, axis=0)


# return a new set of samples drawn from log-likelihood distribution LL
def resample_particles(X, LL):
    N = X.shape[1]
    LL = np.exp(LL - np.max(LL))
    LL /= np.sum(LL)
    j = np.random.choice(N, N, replace=True, p=LL)
    return X[:, j]


def main(f):
    np.random.seed(1)
    # bg = cv2.imread("satview.png")
    bg = cv2.imread("trackmap.jpg")

    Np = 300
    X = np.zeros((3, Np))
    # X[:2] = 100 * np.random.rand(2, Np)
    # X[1] -= 400
    X[:2] = 30*np.random.randn(2, Np)
    X[2] = np.random.randn(Np) * 0.2
    tstamp = None
    last_wheels = None

    done = False
    i = 0
    if VIDEO:
        # vidout = cv2.VideoWriter("particlefilter.h264", cv2.VideoWriter_fourcc(
        #    'X', '2', '6', '4'), 30, (bg.shape[1], bg.shape[0]), True)
        vidout = cv2.VideoWriter("particlefilter.h264", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 30, (640, 480), True)

    while not done:
        ok, frame = recordreader.read_frame(f)
        if not ok:
            break

        _, throttle, steering, accel, gyro, servo, wheels, periods, yuv = frame
        if tstamp is None:
            tstamp = frame[0] - 1.0 / 30
        if last_wheels is None:
            last_wheels = wheels
        ts = frame[0]
        dt = frame[0] - tstamp
        if dt > 0.1:
            print 'WARNING: frame', i, 'has a', dt, 'second gap'
        tstamp = ts
        gyroz = gyro[2]  # we only need the yaw rate from the gyro
        dw = wheels - last_wheels
        last_wheels = wheels
        print 'wheels', dw, 'gyro', gyroz, 'dt', dt
        ds = 0.25*np.sum(dw)
        step(X, dt, ds, gyroz)

        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        mapview = bg.copy()
        # mapview = 200*np.ones(bg.shape, np.uint8)

        # draw all particles
        xi = np.uint32(XOFF + X[0]/a)
        xj = np.uint32(YOFF - X[1]/a)
        xin = (xi >= 0) & (xi < mapview.shape[1]) & (xj >= 0) & (xj < mapview.shape[0])
        mapview[xj[xin], xi[xin], :] = 0
        mapview[xj[xin], xi[xin], 1] = 255
        mapview[xj[xin], xi[xin], 2] = 255

        # draw mean/covariance also
        x = np.mean(X, axis=1)
        P = np.cov(X)
        print "%d: %f %f %f" % (i, x[0], x[1], x[2])

        x0, y0 = x[:2] / a
        dx, dy = 20*np.cos(x[2]), 20*np.sin(x[2])
        U, V, _ = np.linalg.svd(P[:2, :2])
        axes = np.sqrt(V)
        angle = -np.arctan2(U[0, 1], U[0, 0]) * 180 / np.pi
        cv2.ellipse(mapview, (XOFF+int(x0), YOFF-int(y0)), (int(axes[0]), int(axes[1])),
                    angle, 0, 360, (0, 0, 220), 1)
        cv2.circle(mapview, (XOFF+int(x0), YOFF-int(y0)), 3, (0, 0, 220), 2)
        cv2.line(mapview, (XOFF+int(x0), YOFF-int(y0)), (XOFF+int(x0+dx), YOFF-int(y0+dy)), (0, 0, 220), 2)
        for l in range(len(L)):
            lxy = (XOFF+int(L[l, 0]/a), YOFF-int(L[l, 1]/a))
            cv2.circle(mapview, lxy, 3, (0, 128, 255), 3)
            cv2.putText(mapview, "%d" % l, (lxy[0] + 3, lxy[1] + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

        conecenters, origcenters, _ = coneclassify.classify(yuv, gyroz)
        for n, z in enumerate(conecenters):
            # mark the cone on the original view
            p = origcenters[n]
            cv2.circle(bgr, (int(p[0]), int(p[1])), 8, (255, 200, 0), cv2.FILLED)

            zz = np.arctan(z[0])
            j, LL = likeliest_lm(X, L, zz)

            # resample the particles based on their landmark likelihood
            X = resample_particles(X, LL)

            # we could also update the landmarks at this point

            # TODO: visualize the various landmarks being hit by the various particles
            # we could draw thicker lines for more hits and thinner for fewer

            # cv2.line(mapview, (XOFF+int(x0), YOFF-int(y0)), (XOFF+int(ll[0]), YOFF-int(ll[1])), (255,128,0), 1)
            # ll = L[j]
            # x, P, LL = ekf.update_lm_bearing(x, P, -zz, ll[0], ll[1], R)

        cv2.putText(mapview, "%d" % i, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        if VIDEO:
            s = mapview[::2, ::2].shape
            bgr[-s[0]:, -s[1]:] = mapview[::2, ::2]

        annotate.draw_speed(bgr, tstamp, wheels, periods)
        annotate.draw_steering(bgr, steering, servo, center=(200, 420))
        # TODO: also annotate gyroz and lateral accel

        if VIDEO:
            s = (bg.shape[1] - 320) // 2
            vidout.write(bgr)
        else:
            cv2.imshow("map", mapview)
            cv2.imshow("raw", bgr)
            k = cv2.waitKey()
            if k == ord('q'):
                break

        i += 1


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print "need input!\n%s [cycloid-yyyymmdd-hhmmss.rec]" % sys.argv[0]
        sys.exit(1)

    f = open(sys.argv[1])
    main(f)
    f.close()
