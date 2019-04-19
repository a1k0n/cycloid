# particle filter coneslam (just using FastSLAM algorithm but without any trees
# as we have a relatively small number of landmarks)
import numpy as np
import cv2

import annotate
import recordreader
import caldata
import params
import time


VIDEO = False
np.set_printoptions(suppress=True)

# length from back axle to camera in encoder ticks (2cm)
carlength = 0  # 12*.0254/.02


def read_landmarks():
    L = None
    f = open("lm.txt", "r")
    home = np.zeros(3)
    i = 0
    for line in f:
        if line.strip() == '':
            continue
        if line[:4] == "home":
            home[:] = list(map(float, line.strip().split()[1:]))
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
            T.append(list(map(float, line)))
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

    heading = (theta0+theta1)/2.0
    S = np.sin(heading)
    C = np.cos(heading)

    dx = ds + pseudorandn(N) * params.NOISE_LONG * ds * dt
    dy = pseudorandn(N) * params.NOISE_LAT * ds * dt

    X[0] += dx*C - dy*S
    X[1] += dx*S + dy*C
    X[2] = theta1


def likelihood(X, acts):
    angratio = len(acts)/(2*np.pi)  # range of angles, 0..2*pi mapped to 0..len(acts)
    A = np.cumsum(np.concatenate([acts, acts]))
    S, C = np.sin(X[2]), np.cos(X[2])
    dxy = L[:, :, None] - X[:2]
    # counter-rotate each landmark into each particle's frame (y, z)
    z = dxy[:, 0]*C + dxy[:, 1]*S
    y = -dxy[:, 0]*S + dxy[:, 1]*C
    # distance to cone
    d = np.linalg.norm(dxy, axis=1)
    # visible cone angle (radius)
    visibleradius = angratio*np.arcsin(np.minimum(params.CONE_RADIUS/d, 1))
    # expected angle to center of cone for each particle
    coneangle = angratio*np.arctan2(y, z)
    # ...and the range of angles
    c0 = np.round(coneangle - visibleradius).astype(np.int)
    c1 = np.round(coneangle + visibleradius).astype(np.int)
    # move negative angles back up into second copy of table
    shift = c0 < 0
    c0[shift] += len(acts)
    c1[shift] += len(acts)
    # finally use the cumsum to get the sum of activations for expected
    # cone angle ranges
    LL = np.sum((A[c1] - A[c0]), axis=0) * params.PF_TEMP
    # LL *= -len(LL) / (params.PF_VAR * np.sum(LL) + 1)
    return LL, coneangle


# generate remap table for fisheye camera calibration
# generates table for a wxh image
# for a given latitude range (specified in pixels above horizon / equator)
def camcal(w, h, lat0, lat1, lon0=0, lon1=2*np.pi):
    # assume camera calibration was for native raspberry pi camera resolution,
    # rescale as necessary
    scale = h / 1944.0
    camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
    dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
    fx, fy = np.diag(camera_matrix)[:2] * scale
    cx, cy = camera_matrix[:2, 2] * scale
    k1 = dist_coeffs[0]

    theta = np.pi/2 + np.arange(-lat0, -lat1) / fx
    theta = theta*(1 + k1*theta**2)
    midtheta = np.pi/2*(1 + k1*(np.pi/2)**2)
    npix = int(2 * np.pi * fx * midtheta + 0.5)
    t = np.linspace(lon0, lon1, npix)[:-1]
    # offset by -90 degrees for camera orientation
    x = fx*np.outer(theta, np.sin(t)) + cx
    y = -fy*np.outer(theta, np.cos(t)) + cy
    return np.stack([x, y])[:, :-1].transpose(1, 2, 0).astype(np.float32)


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


def main(f, VIDEO, interactive):
    np.random.seed(1)
    # bg = cv2.imread("trackmap.jpg")
    # bg = cv2.imread("drtrack-2cm.png")
    bg = cv2.imread("bball-2cm.png")
    # bg = cv2.imread("cl.png")
    # bg = cv2.imread("voyage-top.png")

    m1 = camcal(320, 240, 10, -5)
    ym1, ym2 = cv2.convertMaps(m1*2, None, cv2.CV_16SC2)
    uvm1, uvm2 = cv2.convertMaps(m1, None, cv2.CV_16SC2)

    def yuvremap(yuv):
        # assumes 640x480 YUV420 image
        y = cv2.remap(yuv[:480], ym1, ym2, cv2.INTER_LINEAR)
        u = cv2.remap(yuv[480:480+120].reshape((240, 320)), uvm1, uvm2, cv2.INTER_LINEAR)
        v = cv2.remap(yuv[480+120:].reshape((240, 320)), uvm1, uvm2, cv2.INTER_LINEAR)
        return np.stack([y, u, v]).transpose(1, 2, 0)

    Np = 300
    X = np.zeros((4, Np))
    # X[:2] = 100 * np.random.rand(2, Np)
    # X[1] -= 400
    X[0] = 4*0.125*pseudorandn(Np)
    X[1] = 4*0.125*pseudorandn(Np)
    X[2] = pseudorandn(Np) * 0.1
    X.T[:, :3] += Lhome
    tstamp = None
    last_wheels = None

    done = False
    i = 0
    if VIDEO is not None:
        #vidout = cv2.VideoWriter(VIDEO, cv2.VideoWriter_fourcc(
        #   'X', '2', '6', '4'), 30, (bg.shape[1], bg.shape[0]), True)
        vidout = cv2.VideoWriter(VIDEO, cv2.VideoWriter_fourcc(
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

        throttle, steering, accel, gyro, servo, wheels, periods = framedata['carstate']
        savedparticles = framedata['particles']
        yuv = framedata['yuv420']
        ts = framedata['tstamp']
        if tstamp is None:
            tstamp = ts - 1.0 / 30
        if last_wheels is None:
            last_wheels = wheels
        dt = ts - tstamp
        if dt > 0.1:
            print('WARNING: frame', i, 'has a', dt, 'second gap')
        tstamp = ts
        tsfrac = tstamp - int(tstamp)
        tstring = time.strftime("%H:%M:%S.", time.localtime(tstamp)) + "%02d" % (tsfrac*100)
        gyroz = gyro[2]  # we only need the yaw rate from the gyro
        dw = wheels - last_wheels
        last_wheels = wheels
        # print('wheels', dw, 'periods', periods, 'gyro', gyroz, 'dt', dt)
        ds = np.sum(dw) * params.WHEEL_TICK_LENGTH / params.NUM_ENCODERS
        vest1 = np.mean(periods[:params.NUM_ENCODERS])
        if vest1 != 0:
            vest1 = params.WHEEL_TICK_LENGTH * 1e6 / vest1
        vest2 += 0.2*(ds / dt - vest2)
        # ds = 0.5*np.sum(dw[2:])
        # w = v k
        # a = v^2 k = v w
        # print('accel', accel, ' expected ', gyroz * vest1 / 9.8, 'v', vest1, vest2, accel[0]*dt * 9.8)
        step(X, dt, ds, gyroz, vest2, -accel[1]*9.8)

        if False:
            Am = 0.8*Am + 0.2*accel[1]*9.8
            Ae = 0.8*Ae + 0.2*ds*gyroz/dt*0.02
            Vlat = 0.8*Vlat + dt*accel[1]*9.8 - ds*gyroz
            print('alat', accel[1]*9.8, 'estimated', ds*gyroz/dt*0.02)
            print('Vlat estimate', Vlat)
            print('measured alat', Am, 'estimated alat', Ae)

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
        # print("%d: %f %f %f" % (i, x[0], x[1], x[2]))

        x0, y0 = x[:2] / a
        dx, dy = 20*np.cos(x[2]), 20*np.sin(x[2])
        U, V, _ = np.linalg.svd(P[:2, :2])
        axes = np.sqrt(V) / a
        angle = -np.arctan2(U[0, 1], U[0, 0]) * 180 / np.pi
        cv2.ellipse(mapview, (XOFF+int(x0), YOFF-int(y0)), (int(axes[0]), int(axes[1])),
                    angle, 0, 360, (0, 0, 220), 1)
        cv2.circle(mapview, (XOFF+int(x0), YOFF-int(y0)), 3, (0, 0, 220), 2)
        cv2.line(mapview, (XOFF+int(x0), YOFF-int(y0)), (XOFF+int(x0+dx), YOFF-int(y0+dy)), (0, 0, 220), 2)
        for l in range(len(L)):
            lxy = (XOFF+int(L[l, 0]/a), YOFF-int(L[l, 1]/a))
            cv2.circle(mapview, lxy, 3, (0, 128, 255), 3)
            cv2.putText(mapview, "%d" % l, (lxy[0] + 3, lxy[1] + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

        if ds > 0:
            #yuv1 = yuvremap(yuv)
            #uv = yuv1[:, :, 1:3].astype(np.int32) - 128
            #activation = uv[:, :, 1]
            #activation[yuv1[:, :, 1] == 0] = 0
            #activation = np.sum(activation, axis=0)
            activation = framedata['activations'].copy()
            activation[1:] = activation[1:] - activation[:-1]
            # acts = np.cumsum(np.concatenate([activation, activation]))
            # LL = np.sum(acts[framedata['c1']] - acts[framedata['c0']], axis=1) * params.PF_TEMP
            LL, coneangle = likelihood(X, activation)
            totalL += np.sum(LL)
            LL -= np.max(LL)
            X = resample_particles(X, LL)

        cv2.putText(mapview, "%d" % i, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(bgr, tstring, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        if VIDEO is not None:
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
        # print('vf', vf, 'vr', vr, 'vr-vf', vr-vf, 'vr/vf', vr/(vf + 0.001))
        if np.abs(delta) > 0.08:
            # estimate lateral velocity
            vy = (vr*np.cos(delta) + gyroz*a*np.sin(delta) - vf)
            # print('lateral velocity *', np.sin(delta), '=', vy)

        if VIDEO is not None:
            vidout.write(bgr)
        if interactive:
            cv2.imshow("map", mapview)
            cv2.imshow("raw", bgr)
            k = cv2.waitKey()
            if k == ord('q'):
                break
        print('frame', i, 'L', totalL)

        i += 1


if __name__ == '__main__':
    import sys
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument('-v', '--video', type=str, default=None,
            help='Video file to save to')
    p.add_argument('-i', '--interactive', dest='interactive', action='store_true',
            help='Pause after every frame (default true)')
    p.add_argument('-ni', '--no-interactive', dest='interactive', action='store_false')
    p.add_argument('recordfile', type=str, help='Recording file from car')
    args = p.parse_args()

    if args.recordfile is None:
        print("need input!\n%s [cycloid-yyyymmdd-hhmmss.rec]" % sys.argv[0])
        sys.exit(1)

    f = open(args.recordfile, 'rb')
    main(f, args.video, args.interactive)
    f.close()
