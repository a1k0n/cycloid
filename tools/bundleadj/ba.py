import cv2
import numpy as np
import scipy.optimize

import recordreader


WHEELTICK_SCALE = 0.066
CAM_TILT = np.array([0, 22.*np.pi/180., 0])
K = np.load("../../tools/camcal/camera_matrix.npy")
dist = np.load("../../tools/camcal/dist_coeffs.npy")
K[:2] /= 4.05
fx, fy = np.diag(K)[:2]
cx, cy = K[:2, 2]
mapsz = 300  # map size
Z = 14   # map zoom factor
uv = np.mgrid[:480, :640][[1, 0]].transpose(1, 2, 0).astype(np.float32)
ceilmask = ((uv[:, :, 1] - cy)**2 + (uv[:, :, 0] - cx + 60)**2) < (np.pi/2.4 * fx)**2
R = cv2.Rodrigues(CAM_TILT)[0]
pts = cv2.fisheye.undistortPoints(uv[None, ceilmask], K, dist, R=R)

ceilmap = np.zeros((mapsz, mapsz), np.float32)
ceilN = np.ones((mapsz, mapsz), np.float32)
ceilmean = ceilmap / ceilN


def pix2floormap():
    ''' undistortPoints doesn't support points behind the image plane, but we can solve for them '''
    def solvetheta(thetad, k1):
        theta = thetad
        theta += (theta*(k1*theta**2 + 1) - thetad)/(-3*k1*theta**2 - 1)
        theta += (theta*(k1*theta**2 + 1) - thetad)/(-3*k1*theta**2 - 1)
        return theta

    mg = np.mgrid[:480, :640]
    u, v = (mg[1] - cx)/fx, (mg[0] - cy)/fy
    r = np.sqrt(u**2 + v**2)
    a, b = u/r, -v/r
    theta = solvetheta(r, dist[0])
    mask = (theta > np.pi/2) & (theta < np.pi/1.9)
    t = 1.0 / np.tan(theta[mask] - np.pi/2)
    return mask, np.stack([a[mask] * t, b[mask] * t]).T


floormap = np.zeros((mapsz, mapsz, 3), np.float32)
floorN = np.ones((mapsz, mapsz), np.float32) * 1e-3
floormean = floormap / floorN[:, :, None]
floormask, floorpts = pix2floormap()


def Maplookup(x, y, theta):
    S, C = np.sin(theta), np.cos(theta)
    R = np.array([[C, S], [-S, C]])*Z
    p = np.dot(pts[0], R.T) + np.array([x, y])
    pi = p.astype(np.int)
    pt = p - pi
    t00 = (1-pt[:, 1])*(1-pt[:, 0])
    t01 = (1-pt[:, 1])*(pt[:, 0])
    t10 = (pt[:, 1])*(1-pt[:, 0])
    t11 = (pt[:, 1])*(pt[:, 0])
    m = (t00*ceilmean[pi[:, 1], pi[:, 0]+1] +
         t01*ceilmean[pi[:, 1], pi[:, 0]+1] +
         t10*ceilmean[pi[:, 1]+1, pi[:, 0]+1] +
         t11*ceilmean[pi[:, 1]+1, pi[:, 0]+1])
    return m


def Mapupdate(xi, yi, theta, gray):
    S, C = np.sin(theta), np.cos(theta)
    R = np.array([[C, S], [-S, C]])*Z
    p = np.dot(pts[0], R.T) + np.array([xi, yi])
    pi = p.astype(np.int)
    pt = p - pi
    t00 = (1-pt[:, 1])*(1-pt[:, 0])
    t01 = (1-pt[:, 1])*(pt[:, 0])
    t10 = (pt[:, 1])*(1-pt[:, 0])
    t11 = (pt[:, 1])*(pt[:, 0])
    idxs = pi[:, 1] * mapsz + pi[:, 0]
    ceilN[:] += np.bincount(idxs, t00.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    ceilN[:] += np.bincount(idxs+1, t01.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    ceilN[:] += np.bincount(idxs+mapsz, t10.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    ceilN[:] += np.bincount(idxs+mapsz+1, t11.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    mask = ceilmask
    ceilmap[:] += np.bincount(idxs, t00*gray[mask], mapsz*mapsz).reshape(mapsz, mapsz)
    ceilmap[:] += np.bincount(idxs+1, t01*gray[mask], mapsz*mapsz).reshape(mapsz, mapsz)
    ceilmap[:] += np.bincount(idxs+mapsz, t10*gray[mask], mapsz*mapsz).reshape(mapsz, mapsz)
    ceilmap[:] += np.bincount(idxs+mapsz+1, t11*gray[mask], mapsz*mapsz).reshape(mapsz, mapsz)
    ceilmean[:] = ceilmap / ceilN


def Floorupdate(xi, yi, theta, bgr):
    S, C = np.sin(-theta), np.cos(-theta)
    R = np.array([[C, S], [-S, C]])
    p = np.dot(floorpts, R.T) + np.array([xi, yi])
    mask2 = (p[:, 0] >= 0) & (p[:, 1] >= 0) & (p[:, 0] < mapsz-1) & (p[:, 1] < mapsz-1)
    p = p[mask2]
    pi = p.astype(np.int)
    pt = p - pi
    t00 = (1-pt[:, 1])*(1-pt[:, 0])
    t01 = (1-pt[:, 1])*(pt[:, 0])
    t10 = (pt[:, 1])*(1-pt[:, 0])
    t11 = (pt[:, 1])*(pt[:, 0])
    idxs = pi[:, 1] * mapsz + pi[:, 0]
    floorN[:] += np.bincount(idxs, t00.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    floorN[:] += np.bincount(idxs+1, t01.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    floorN[:] += np.bincount(idxs+mapsz, t10.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    floorN[:] += np.bincount(idxs+mapsz+1, t11.reshape(-1), mapsz*mapsz).reshape(mapsz, mapsz)
    mask = floormask
    for i in range(3):
        floormap[:, :, i] += np.bincount(idxs, t00*bgr[mask, i][mask2], mapsz*mapsz).reshape(mapsz, mapsz)
        floormap[:, :, i] += np.bincount(idxs+1, t01*bgr[mask, i][mask2], mapsz*mapsz).reshape(mapsz, mapsz)
        floormap[:, :, i] += np.bincount(idxs+mapsz, t10*bgr[mask, i][mask2], mapsz*mapsz).reshape(mapsz, mapsz)
        floormap[:, :, i] += np.bincount(idxs+mapsz+1, t11*bgr[mask, i][mask2], mapsz*mapsz).reshape(mapsz, mapsz)
    floormean[:] = floormap / floorN[:, :, None]


def main(fname, skips=0):
    f = open(fname, 'rb')
    ts0 = None
    theta = -0.09
    ri = recordreader.RecordIterator(f)
    for _ in range(skips):
        ri.__next__()

    xi, yi = mapsz/2, mapsz/2

    ptrange = np.max(np.linalg.norm(pts[0], axis=1))
    firstiter = False
    if firstiter:
        poses = []
    else:
        poses = np.load("poses.npy")

    frameno = 0

    vidout = None

    if False:
        vidout = cv2.VideoWriter("ba.mp4", cv2.VideoWriter_fourcc(
            'X', '2', '6', '4'), 30, (640, 480), True)

    lastw = None
    for frame in ri:
        gray = frame['yuv420'][:480]
        bgr = cv2.cvtColor(frame['yuv420'], cv2.COLOR_YUV2BGR_I420)
        if ts0 is None:
            ts0 = frame['tstamp'] - 1.0/30
        ts = frame['tstamp']
        dt = ts - ts0
        ts0 = ts

        # throttle, steering, accel, gyro, servo, wheels, periods
        gz = frame['carstate'][3][2]
        wheels = frame['carstate'][5]
        ds = 0
        if lastw is not None:
            ds = (wheels - lastw)[0]
        lastw = wheels

        theta -= gz*dt

        S, C = np.sin(theta), np.cos(theta)
        # now solve for x, y, theta
        xi -= ds*WHEELTICK_SCALE*S
        yi -= ds*WHEELTICK_SCALE*C

        gm = gray[ceilmask]

        def L(X):
            x, y, theta = X
            # m, n = Maplookup(x, y, theta)
            # i really thought that this would work better,
            # but it totally doesn't
            # return (gray[mask].astype(np.float32)*n - m)

            # odometry constraint

            return np.concatenate([(gm - Maplookup(x, y, theta)) / 170.0,
                                   1e-3*np.array([xi - x, yi - y])])

        if False:
            L0 = np.sum(L([xi, yi, theta])**2)
            Lx = np.sum(L([xi+1, yi, theta])**2)
            Ly = np.sum(L([xi, yi+1, theta])**2)
            Lt = np.sum(L([xi, yi, theta+0.01])**2)
            print("jac: ", (Lx - L0), (Ly - L0), (Lt - L0)/0.01)

        if firstiter:
            lb = ptrange*Z
            ub = mapsz - 2 - ptrange*Z
            soln = scipy.optimize.least_squares(
                L, x0=np.array([xi, yi, theta]), loss='soft_l1',
                bounds=([lb, lb, theta - 0.1], [ub, ub, theta + 0.1]),
                verbose=2)
            print(soln)
            xi, yi, theta = soln['x']

            poses.append([xi, yi, theta, dt, ds, gz])
        else:
            xi, yi, theta = poses[frameno][:3]
        frameno += 1

        Mapupdate(xi, yi, theta, gray)
        Floorupdate(mapsz - xi, yi, theta, bgr)

        S, C = np.sin(theta), np.cos(theta)
        disp = ceilmean.astype(np.uint8)
        disp = cv2.resize(disp, dsize=(mapsz, mapsz),
                          interpolation=cv2.INTER_NEAREST)
        for i in range(1, len(poses[:frameno])):
            p0 = poses[i-1]
            p1 = poses[i]
            cv2.line(disp, (int(32*p0[0]), int(32*p0[1])),
                     (int(32*p1[0]), int(32*p1[1])), 255, 1,
                     cv2.LINE_AA, 5)
        cv2.circle(disp, (int(16*xi), int(16*yi)), 30, 0, -2, cv2.LINE_AA, 4)
        cv2.line(disp, (int(32*xi), int(32*yi)),
                 (int(32*xi - 160*S), int(32*yi - 160*C)), 170, 1,
                 cv2.LINE_AA, 5)

        # mark optical center
        cv2.circle(bgr, (int(16*cx), int(16*cy)), 50, (170, 255, 170), 1, cv2.LINE_AA, 4)

        if vidout is not None:
            bgr[-mapsz:, -mapsz:, :] = disp[:, :, None]
            vidout.write(bgr)

        if True:
            cv2.imshow("ceilmap", disp)
            bgr[floormask] = np.clip(bgr[floormask] + 100, 0, 255)
            cv2.imshow("bgr", bgr)
            cv2.imshow("floor", floormean.astype(np.uint8))

            k = cv2.waitKey(1)
            if k == ord('q'):
                break

    cv2.imshow("ceilmap", disp)
    cv2.imshow("bgr", bgr)
    # pause before quitting
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    f.close()
    print("saving ceiling map")
    np.save("ceilmapY.npy", ceilmap)
    np.save("ceilmapN.npy", ceilN)
    np.save("poses.npy", np.array(poses))


if __name__ == '__main__':
    import sys
    main(sys.argv[1], int(sys.argv[2]))
