# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import pickle
import cv2
import recordreader

from numpy import sin, cos, arctan2 as atan2, pi, log


VIDEO = False
KEYWAIT = 1  # 1 for autoplay, 0 for frame-advance

# first experiment with unknown landmarks: we know the *number* of landmarks,
# so initially just put all landmarks at the origin with a huge stddev; they'll
# each become the maximum likelihood pick in turn as new cones are seen

NUM_PARTICLES = 1000
NUM_CONES = 4
NOISE_ANGULAR = 0.4
NOISE_LONG = 20
NOISE_LAT = 1
NOISE_BEARING = 0.4

NEW_LM_THRESH = -2.8
NEW_LM_DIST = 300
NEW_LM_COV = 600**2

a = 0.875  # image scale factor
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


def pf_update(X, L, l_px, r):
    # Do a particle filter update: first, evaluate likelihood of every
    # landmark for every particle
    # X.shape is [3, NumParticles] (x, y, theta)
    # L.shape is [5, NumLandmarks, NumParticles] (x, y, p11, p12, p22)
    Np = X.shape[1]
    p_x = X[0]
    p_y = X[1]
    theta = X[2]
    l_x = L[0]
    l_y = L[1]
    p11 = L[2]
    p12 = L[3]
    p22 = L[4]

    k0 = sin(theta)
    k1 = l_x - p_x
    k2 = k0*k1
    k3 = cos(theta)
    k4 = l_y - p_y
    k5 = k3*k4
    k6 = k2 - k5
    k7 = k0*k4 + k1*k3
    k8 = l_px - atan2(k6, k7)
    k9 = 1/(k6**2 + k7**2)
    k10 = k7*k9
    k11 = k9*(-k2 + k5)
    k12 = k0*k10 + k11*k3
    k13 = k0*k11 - k10*k3
    k14 = k12*(k12*p11 + k13*p12) + k13*(k12*p12 + k13*p22) + r
    LL = -0.5*log(4*pi**2*k14) - k8**2/k14

    # get the maximum likelihood
    i = np.argmax(LL, axis=0)
    j = np.arange(Np)
    # if the best we can do is still bad, make a new landmark
    newlms = LL[i, j] <= NEW_LM_THRESH
    oldlms = LL[i, j] > NEW_LM_THRESH

    LL = LL[i, j]
    i = i[oldlms]
    j = j[oldlms]

    y_k = k8[i, j]
    S = k14[i, j]
    H1 = k12[i, j]
    H2 = k13[i, j]

    p11 = L[2, i, j]
    p12 = L[3, i, j]
    p22 = L[4, i, j]

    # now compute the EKF update for each maximum-likelihood landmark position
    k0 = 1/S
    k1 = H2*p12
    k2 = k0*(H1*p11 + k1)
    k3 = H1*p12
    k4 = H2*p22
    k5 = k0*(k3 + k4)
    k6 = H1*k2 - 1
    L[0, i, j] += k2*y_k
    L[1, i, j] += k5*y_k
    L[2, i, j] = -k1*k2 - k6*p11
    L[3, i, j] = -k2*k4 - k6*p12
    L[4, i, j] = -k3*k5 - p22*(H2*k5 - 1)

    # new landmark position / covariance
    if np.any(newlms):
        newlmi = np.argmax(L[0, :, newlms] == -1000, axis=1)
        L[0, newlmi, newlms] = p_x[newlms] + cos(theta[newlms]-l_px)*NEW_LM_DIST
        L[1, newlmi, newlms] = p_y[newlms] + sin(theta[newlms]-l_px)*NEW_LM_DIST
        L[2, newlmi, newlms] = NEW_LM_COV
        L[3, newlmi, newlms] = 0
        L[4, newlmi, newlms] = NEW_LM_COV
        print "%d particles creating new landmark @ %f %f" % (
            len(newlmi), np.mean(L[0, newlmi, newlms]), np.mean(L[1, newlmi, newlms]))

    return LL


# return a new set of samples drawn from log-likelihood distribution LL
def resample_particles(X, L, LL):
    N = X.shape[1]
    LL = np.exp(LL - np.max(LL))
    LL /= np.sum(LL)
    j = np.random.choice(N, N, replace=True, p=LL)
    return X[:, j], L[:, :, j]


def draw_gaussian(img, x, P, color):
    x0, y0 = x/a
    U, V, _ = np.linalg.svd(P)
    axes = np.sqrt(V)
    angle = -np.arctan2(U[0, 1], U[0, 0]) * 180 / np.pi
    cv2.ellipse(img, (123+int(x0), 475-int(y0)), (int(axes[0]), int(axes[1])),
                angle, 0, 360, color, 1)


def main(data, f):
    np.random.seed(1)
    bg = cv2.imread("satview.png")

    camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
    dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
    camera_matrix[:2] /= 4.  # for 640x480

    X = np.zeros((3, NUM_PARTICLES))
    X[0] = 200
    X[1] = 150
    L = np.zeros((5, NUM_CONES, NUM_PARTICLES))
    L[:2] = np.array([
        [400, 100],
        [400, -100],
        [100, -100],
        [100, 100],
    ]).T[:, :, None]

    L[2] = 300**2
    L[4] = 300**2
    #L[2] = 0
    #L[4] = 0
    #L[:2, 6:8] = (L0.T*a)[:, 6:8, None]
    #L[2, 6:8] = 0.1  # anchor the first seen cone location
    #L[4, 6:8] = 0.1
    tstamp = data[0][0][0] - 1.0 / 30
    last_wheels = data[0][0][6]

    done = False
    i = 0
    if VIDEO:
        vidout = cv2.VideoWriter("particlefilter.h264", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 30, (bg.shape[1], bg.shape[0]), True)

    maxL = L[:, :, 0]
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
        #mapview = bg.copy()
        mapview = 200*np.ones(bg.shape, np.uint8)

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
        draw_gaussian(mapview, x[:2], P[:2, :2], (0, 0, 220))
        cv2.circle(mapview, (123+int(x0), 475-int(y0)), 3, (0, 0, 220), 2)
        cv2.line(mapview, (123+int(x0), 475-int(y0)), (123+int(x0+dx), 475-int(y0+dy)), (0, 0, 220), 2)

        LL = None
        for n, z in enumerate(data[i][1]):
            # mark the cone on the original view
            p = cv2.fisheye.distortPoints(np.array([[z]], np.float32), camera_matrix, dist_coeffs)[0][0]
            cv2.circle(bgr, (int(p[0]), int(p[1])), 8, (255, 200, 0), cv2.FILLED)

            zz = np.arctan(z[0])
            newLL = pf_update(X, L, zz, NOISE_BEARING)
            if LL is None:
                LL = newLL
            else:
                LL += newLL
            # we could also update the landmarks at this point

            # TODO: visualize the various landmarks being hit by the various particles
            # we could draw thicker lines for more hits and thinner for fewer

            # cv2.line(mapview, (123+int(x0), 475-int(y0)), (123+int(ll[0]), 475-int(ll[1])), (255,128,0), 1)
            # ll = L[j]
            # x, P, LL = ekf.update_lm_bearing(x, P, -zz, ll[0], ll[1], R)

        # resample particles after all observations
        if LL is not None:
            maxL = L[:, :, np.argmax(LL)]
            # resample the particles based on their landmark likelihood
            X, L = resample_particles(X, L, LL)

        for l in range(NUM_CONES):
            lxy = (123+int(maxL[0, l]/a), 475-int(maxL[1, l]/a))
            cv2.circle(mapview, lxy, 3, (0, 128, 255), 3)
            P = np.array([[maxL[2, l], maxL[3, l]], [maxL[3, l], maxL[4, l]]])
            draw_gaussian(mapview, maxL[:2, l], P, (0, 128, 255))
            cv2.putText(mapview, "%d" % l, (lxy[0] + 3, lxy[1] + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

        cv2.putText(mapview, "%d" % i, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        if VIDEO:
            mapview[:240, 160:480] = bgr[::2, ::2]
            vidout.write(mapview)
        else:
            cv2.imshow("raw", bgr)
            cv2.imshow("map", mapview)
            k = cv2.waitKey(KEYWAIT)
            if k == ord('q'):
                break

        i += 1
    print 'final landmarks:'
    print maxL.T


if __name__ == '__main__':
    np.set_printoptions(suppress=True)

    #data = pickle.load(open("20180804-194415.cones.pickle"))
    #f = open("home20180804/cycloid-20180804-194415.rec")
    #data = pickle.load(open("20180804-194750.cones.pickle"))
    #f = open("home20180804/cycloid-20180804-194750.rec")
    data = pickle.load(open("20180817-170943.cones.pickle"))
    f = open("home20180817/cycloid-20180817-170943.rec")
    main(data, f)
    f.close()
