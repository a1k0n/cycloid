# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import cv2

import annotate
import recordreader
import coneclassify
import caldata
import params
import time


VIDEO = True
np.set_printoptions(suppress=True)

# length from back axle to camera in encoder ticks (2cm)
carlength = 0  # 12*.0254/.02


def read_landmarks():
    L = None
    f = open("lm.txt")
    home = np.zeros(3)
    i = 0
    for line in f:
        if line.strip() == '':
            continue
        if line[:4] == "home":
            home[:] = map(float, line.strip().split()[1:])
            continue
        if L is None:
            L = np.zeros((int(line.strip()), 2))
        else:
            x, y = map(float, line.strip().split())
            L[i] = [x, y]
            i += 1
    f.close()
    return L, home


def read_track():
    T = []
    f = open("track.txt")
    for line in f:
        line = line.strip().split()
        if len(line) == 5:
            T.append(map(float, line))
    f.close()
    return np.array(T)

L, Lhome = read_landmarks()
Track = read_track()
a = 0.02
XOFF = 0
YOFF = 0


def pseudorandn(N):
    r = np.random.random(N)
    for j in range(1, 6):
        r += np.random.random(N)
    return 2*r - 6


def step(X, dt, ds, gyro_dtheta, v, accel_y):
    N = X.shape[1]
    theta0 = X[2]
    theta1 = theta0 + gyro_dtheta*dt + pseudorandn(N) * params.NOISE_ANGULAR * ds * dt

    heading = X[3]
    # now how much angular velocity does our actual heading have we have given
    # the lateral acceleration? (basically, how much are we understeering?)
    # the accelerometer is noisy as hell, so we can't just integrate it...
    # we need some sort of index of understeeringness
    alpha = pseudorandn(N) * params.NOISE_STEER_s + params.NOISE_STEER_u
    heading += alpha*(theta1 - heading)

    heading = theta0
    S = np.sin(heading)
    C = np.cos(heading)

    dx = ds + pseudorandn(N) * params.NOISE_LONG * ds * dt
    dy = pseudorandn(N) * params.NOISE_LAT * ds * dt

    X[0] += dx*C - dy*S
    X[1] += dx*S + dy*C
    X[2] = theta1
    X[3] = heading


def likeliest_lm(X, L, l):
    # X is [Np, 3]
    # L is [Nl, 2]
    # we need to check Np x Nl combinations

    # dxy[l, :, p] is the relative position between particle p and landmark l
    S, C = np.sin(X[2]), np.cos(X[2])
    dxy = L[:, :, None] - (X[:2] + carlength*np.array([C, S]))
    # rotate each landmark into each particle's frame (y, z)
    z = dxy[:, 0]*C + dxy[:, 1]*S
    y = dxy[:, 0]*S - dxy[:, 1]*C
    # get the relative angle^2
    d = np.linalg.norm(dxy, axis=1)
    # print 'd', d.shape, d
    coneangle = 2*np.arcsin(np.minimum(params.CONE_RADIUS/d, 1))
    # print 'coneangle', coneangle
    angledist = np.maximum(np.abs(np.arctan2(y, z) - l) - coneangle, 0)
    e = np.minimum(angledist**2, params.BOGON_THRESH)
    LL = -params.LM_SELECTIVITY*e

    # normalize probabilities so that resampling among landmarks is fair
    #LL -= np.max(LL)
    #LL -= np.log(np.sum(np.exp(LL))) - np.log(X.shape[0])

    j = np.argmax(LL, axis=0)
    return j, np.max(LL, axis=0)


# return a new set of samples drawn from log-likelihood distribution LL
def resample_particles(X, LL):
    N = X.shape[1]
    LL = np.exp(LL - np.max(LL))
    totalP = np.sum(LL)
    deltaP = totalP / N
    randP = np.random.random() * totalP
    js = []
    j = 0
    for i in range(N):
        while randP > LL[j]:
            randP -= LL[j]
            j += 1
            if j == N:
                j = 0
        js.append(j)
        randP += deltaP
    return X[:, js]


def main(f):
    np.random.seed(1)
    # bg = cv2.imread("trackmap.jpg")
    # bg = cv2.imread("drtrack-2cm.png")
    # bg = cv2.imread("bball-2cm.png")
    bg = cv2.imread("cl.png")
    # bg = cv2.imread("voyage-top.png")

    Np = 300
    X = np.zeros((4, Np))
    # X[:2] = 100 * np.random.rand(2, Np)
    # X[1] -= 400
    X[0] = 0.125*pseudorandn(Np)
    X[1] = 0.125*pseudorandn(Np)
    X[2] = pseudorandn(Np) * 0.1
    print 'Lhome', Lhome
    X.T[:, :3] += Lhome
    X[3] = X[2]
    tstamp = None
    last_wheels = None

    done = False
    i = 0
    if VIDEO:
        # vidout = cv2.VideoWriter("particlefilter.h264", cv2.VideoWriter_fourcc(
        #    'X', '2', '6', '4'), 30, (bg.shape[1], bg.shape[0]), True)
        vidout = cv2.VideoWriter("particlefilter.h264", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 32.4, (640, 480), True)
        if not vidout:
            return 'video out fail?'

    Am, Ae = 0, 0
    Vlat = 0
    totalL = 0
    vest2 = 0
    while not done:
        ok, framedata = recordreader.read_frame(f)
        if not ok:
            break

        throttle, steering, accel, gyro, servo, wheels, periods = framedata[1]
        savedparticles = framedata[2]
        yuv = framedata[-1].reshape((-1, 640))
        if tstamp is None:
            tstamp = framedata[0] - 1.0 / 30
        if last_wheels is None:
            last_wheels = wheels
        ts = framedata[0]
        dt = framedata[0] - tstamp
        if dt > 0.1:
            print 'WARNING: frame', i, 'has a', dt, 'second gap'
        tstamp = ts
        tsfrac = tstamp - int(tstamp)
        tstring = time.strftime("%H:%M:%S.", time.localtime(tstamp)) + "%02d" % (tsfrac*100)
        gyroz = gyro[2]  # we only need the yaw rate from the gyro
        dw = wheels - last_wheels
        last_wheels = wheels
        print 'wheels', dw, 'periods', periods, 'gyro', gyroz, 'dt', dt
        ds = np.sum(dw) * params.WHEEL_TICK_LENGTH / params.NUM_ENCODERS
        vest1 = np.mean(periods[:params.NUM_ENCODERS])
        if vest1 != 0:
            vest1 = params.WHEEL_TICK_LENGTH * 1e6 / vest1
        vest2 += 0.2*(ds / dt - vest2)
        # ds = 0.5*np.sum(dw[2:])
        # w = v k
        # a = v^2 k = v w
        print 'accel', accel, ' expected ', gyroz * vest1 / 9.8, 'v', vest1, vest2, accel[0]*dt * 9.8
        step(X, dt, ds, gyroz, vest2, -accel[1]*9.8)

        if False:
            Am = 0.8*Am + 0.2*accel[1]*9.8
            Ae = 0.8*Ae + 0.2*ds*gyroz/dt*0.02
            Vlat = 0.8*Vlat + dt*accel[1]*9.8 - ds*gyroz
            print 'alat', accel[1]*9.8, 'estimated', ds*gyroz/dt*0.02
            print 'Vlat estimate', Vlat
            print 'measured alat', Am, 'estimated alat', Ae

        yuv = yuv.copy()
        yuv[480+120:] = np.maximum(yuv[480+120:], 128)  # filter out blue
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

        xi = np.uint32(XOFF + savedparticles[:, 0]/.02)
        xj = np.uint32(YOFF - savedparticles[:, 1]/.02)
        xin = (xi >= 0) & (xi < mapview.shape[1]) & (xj >= 0) & (xj < mapview.shape[0])
        mapview[xj[xin], xi[xin], :] = 0
        mapview[xj[xin], xi[xin], 1] = 255
        mapview[xj[xin], xi[xin], 2] = 0

        # draw mean/covariance also
        x = np.mean(X, axis=1)
        P = np.cov(X)
        # print "%d: %f %f %f" % (i, x[0], x[1], x[2])

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

        conecenters, origcenters, cone_acts = coneclassify.classify(yuv, gyroz)
        LLs = np.zeros(Np)
        for n, z in enumerate(conecenters):
            # mark the cone on the original view
            p = origcenters[n]
            cv2.circle(bgr, (int(p[0]), int(p[1])), 8, (255, 200, 0), cv2.FILLED)

            zz = np.arctan(z[0])
            j, LL = likeliest_lm(X, L, zz)
            LLs += LL
            totalL += np.sum(LL)
            # print 'frame', i, 'cone', n, 'LL', np.min(LL), np.mean(LL), '+-', np.std(LL), np.max(LL)

            # we could also update the landmarks at this point

            # TODO: visualize the various landmarks being hit by the various particles
            # we could draw thicker lines for more hits and thinner for fewer

            # cv2.line(mapview, (XOFF+int(x0), YOFF-int(y0)), (XOFF+int(ll[0]), YOFF-int(ll[1])), (255,128,0), 1)
            # ll = L[j]
            # x, P, LL = ekf.update_lm_bearing(x, P, -zz, ll[0], ll[1], R)

        bgr[params.vpy, ::2][cone_acts] = 255
        bgr[params.vpy, 1::2][cone_acts] = 255
        bgr[params.vpy+1, ::2][cone_acts] = 255
        bgr[params.vpy+1, 1::2][cone_acts] = 255

        if ds > 0 and len(conecenters) > 0:
            # resample the particles based on their landmark likelihood
            X = resample_particles(X, LL)

        cv2.putText(mapview, "%d" % i, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(bgr, tstring, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        if VIDEO:
            #s = mapview[::2, ::2].shape
            #bgr[-s[0]:, -s[1]:] = mapview[::2, ::2]
            pass

        # draw particle map on front view too
        xi = np.uint32(400 + savedparticles[:, 0]/.08)
        xj = np.uint32(-savedparticles[:, 1]/.08)
        xin = (xi >= 0) & (xi < bgr.shape[1]) & (xj >= 0) & (xj < bgr.shape[0])
        bgr[xj[xin], xi[xin], :] = 0
        bgr[xj[xin], xi[xin], 1] = 255
        for l in range(len(L)):
            lxy = (400+int(L[l, 0]/.08), 0-int(L[l, 1]/.08))
            cv2.circle(bgr, lxy, 2, (0, 128, 255), 2)
        for t in range(0, len(Track), 2):
            txy1 = (400+int(Track[t, 0]/.08), 0-int(Track[t, 1]/.08))
            txy2 = (400+int(Track[t+1, 0]/.08), 0-int(Track[t+1, 1]/.08))
            cv2.line(bgr, txy1, txy2, (190, 190, 190), 1)

        annotate.draw_throttle(bgr, throttle)
        annotate.draw_speed(bgr, tstamp, wheels, periods)
        annotate.draw_steering(bgr, steering, servo, center=(200, 420))
        annotate.draw_accelerometer(bgr, accel, gyro, center=(320, 420))
        # TODO: also annotate gyroz and lateral accel

        # get steering angle, front and rear wheel velocities
        delta = caldata.wheel_angle(servo)
        vf = 0.5*np.sum(dw[:2])
        vr = 0.5*np.sum(dw[2:])
        # print 'vf', vf, 'vr', vr, 'vr-vf', vr-vf, 'vr/vf', vr/(vf + 0.001)
        if np.abs(delta) > 0.08:
            # estimate lateral velocity
            vy = (vr*np.cos(delta) + gyroz*a*np.sin(delta) - vf)
            # print 'lateral velocity *', np.sin(delta), '=', vy

        if VIDEO:
            vidout.write(bgr)
        else:
            cv2.imshow("map", mapview)
            cv2.imshow("raw", bgr)
            k = cv2.waitKey()
            if k == ord('q'):
                break
        print 'frame', i, 'L', totalL

        i += 1


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print "need input!\n%s [cycloid-yyyymmdd-hhmmss.rec]" % sys.argv[0]
        sys.exit(1)

    f = open(sys.argv[1])
    main(f)
    f.close()
