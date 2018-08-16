# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import pickle
import cv2
import recordreader


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

L0[:, 1] -= 389
a = 0.875  # fudge factor for scale w.r.t. ticks
L = L0*a

NOISE_ANGULAR = 0.4
NOISE_LONG = 20
NOISE_LAT = 1
LM_SELECTIVITY = 20  # 250


def step(X, dt, encoder_dx, gyro_dtheta):
    N = X.shape[1]
    theta0 = X[2]
    theta1 = theta0 + gyro_dtheta*dt + np.random.randn(N) * NOISE_ANGULAR * dt

    S = np.sin((theta0 + theta1) * 0.5)
    C = np.cos((theta0 + theta1) * 0.5)

    dx = encoder_dx + np.random.randn(N) * NOISE_LONG * dt
    dy = np.random.randn(N) * NOISE_LAT * dt

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
    # FIXME: do this without arctans, just dot projections
    # this subtraction could be problematic but only for landmarks behind us

    # 40 found by tuning total likelihoods
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


def main(data, f):
    np.random.seed(1)
    bg = cv2.imread("satview.png")

    camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
    dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
    camera_matrix[:2] /= 4.  # for 640x480

    Np = 1000
    X = np.zeros((3, Np))
    X[:2] = 800 * np.random.rand(2, Np)
    X[1] -= 400
    X[2] = np.random.rand(Np) * 0.2 - 0.1
    tstamp = data[0][0][0] - 1.0 / 30
    last_wheels = data[0][0][6]

    done = False
    i = 0
    if VIDEO:
        vidout = cv2.VideoWriter("particlefilter.h264", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 30, (bg.shape[1], bg.shape[0]), True)

    while not done and i < len(data):
        ok, frame = recordreader.read_frame(f)
        if not ok:
            break
        d = data[i]

        ts = d[0][0]
        dt = ts - tstamp
        if dt > 0.1:
            print 'WARNING: frame', i, 'has a', dt, 'second gap'
        tstamp = ts
        gyro = d[0][4][2]
        dw = d[0][6] - last_wheels
        last_wheels = d[0][6]
        ds = 0.5 * np.sum(dw[:2])  # front wheels only!
        step(X, dt, ds, gyro)

        bgr = cv2.cvtColor(frame[-1], cv2.COLOR_YUV2BGR_I420)
        mapview = bg.copy()

        # draw all particles
        xi = np.uint32(123 + X[0]/a)
        xj = np.uint32(475 - X[1]/a)
        xin = (xi >= 0) & (xi < mapview.shape[1]) & (xj >= 0) & (xj < mapview.shape[0])
        mapview[xj[xin], xi[xin], :] = 0
        mapview[xj[xin], xi[xin], 1] = 255
        mapview[xj[xin], xi[xin], 2] = 255

        # draw mean/covariance also
        x = np.mean(X, axis=1)
        P = np.cov(X)

        x0, y0 = x[:2] / a
        dx, dy = 20*np.cos(x[2]), 20*np.sin(x[2])
        U, V, _ = np.linalg.svd(P[:2, :2])
        axes = np.sqrt(V)
        angle = -np.arctan2(U[0, 1], U[0, 0]) * 180 / np.pi
        cv2.ellipse(mapview, (123+int(x0), 475-int(y0)), (int(axes[0]), int(axes[1])),
                    angle, 0, 360, (0, 0, 220), 1)
        cv2.circle(mapview, (123+int(x0), 475-int(y0)), 3, (0, 0, 220), 2)
        cv2.line(mapview, (123+int(x0), 475-int(y0)), (123+int(x0+dx), 475-int(y0+dy)), (0, 0, 220), 2)
        for l in range(len(L)):
            lxy = (123+int(L[l, 0]/a), 475-int(L[l, 1]/a))
            cv2.circle(mapview, lxy, 3, (0, 128, 255), 3)
            cv2.putText(mapview, "%d" % l, (lxy[0] + 3, lxy[1] + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

        for n, z in enumerate(data[i][1]):
            # mark the cone on the original view
            p = cv2.fisheye.distortPoints(np.array([[z]], np.float32), camera_matrix, dist_coeffs)[0][0]
            cv2.circle(bgr, (int(p[0]), int(p[1])), 8, (255, 200, 0), cv2.FILLED)

            zz = np.arctan(z[0])
            j, LL = likeliest_lm(X, L, zz)

            # resample the particles based on their landmark likelihood
            X = resample_particles(X, LL)

            # we could also update the landmarks at this point

            # TODO: visualize the various landmarks being hit by the various particles
            # we could draw thicker lines for more hits and thinner for fewer

            # cv2.line(mapview, (123+int(x0), 475-int(y0)), (123+int(ll[0]), 475-int(ll[1])), (255,128,0), 1)
            # ll = L[j]
            # x, P, LL = ekf.update_lm_bearing(x, P, -zz, ll[0], ll[1], R)

        cv2.putText(mapview, "%d" % i, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        if VIDEO:
            mapview[:240, 160:480] = bgr[::2, ::2]
            vidout.write(mapview)
        else:
            cv2.imshow("raw", bgr)
            cv2.imshow("map", mapview)
            k = cv2.waitKey()
            if k == ord('q'):
                break

        i += 1


if __name__ == '__main__':
    #data = pickle.load(open("20180804-194625.cones.pickle"))
    #f = open("home20180804/cycloid-20180804-194625.rec")
    data = pickle.load(open("20180804-194304.cones.pickle"))
    f = open("home20180804/cycloid-20180804-194304.rec")
    main(data, f)
    f.close()
