import numpy as np
import cv2


CAM_TILT = np.array([0, 22. * np.pi / 180., 0])
CEIL_HEIGHT = 8.25
X_GRID = 10/CEIL_HEIGHT
Y_GRID = 12/CEIL_HEIGHT

# camera stands 90mm off the ground, which is almost exactly 0.3 feet (4")
# however, this value was determined empirically to work and isn't
# based on a measurement
FLOOR_HEIGHT = 0.30
K, dist = None, None


def undistortMap():
    global K, dist
    K = np.load("../../tools/camcal/camera_matrix.npy")
    dist = np.load("../../tools/camcal/dist_coeffs.npy")
    K[:2] /= 4.05
    fx, fy = np.diag(K)[:2]
    cx, cy = K[:2, 2]
    uv = np.mgrid[:480, :640][[1, 0]].transpose(1, 2, 0).astype(np.float32)
    R = cv2.Rodrigues(CAM_TILT)[0]
    origpts = cv2.fisheye.undistortPoints(uv, K=K, D=dist)
    pts = np.stack([origpts[:, :, 0], origpts[:, :, 1], np.ones((480, 640))])
    return np.dot(R, pts.transpose(1, 0, 2)), origpts


def ceillut():
    pts, origpts = undistortMap()
    centerlimit = 8
    ceillimit = 3
    ceilmask = ((pts[2] > 0)
                & (np.sum(pts[:2]**2, axis=0) / pts[2]**2 < ceillimit**2)
                & (np.sum(origpts**2, axis=2) < centerlimit**2))
    pts = pts[:2, ceilmask] / pts[2, ceilmask]
    return ceilmask, pts


def floorlut(sampleframe):
    floorlut, origpts = undistortMap()
    fxy = floorlut[:2] / floorlut[2]
    floormask = (floorlut[2] < 0) & (np.sum(fxy**2, axis=0) < 16**2) & (
        np.sum(origpts**2, axis=2) < 6**2) & (sampleframe > 50)
    fpts = fxy[:, floormask]
    return floormask, fpts


def moddist(x, q):
    return (x+q/2) % q - q/2


def match(gray, ceilmask, pts):
    return pts[:, gray[ceilmask] > 240]


def Rmat(theta):
    S, C = np.sin(theta), np.cos(theta)
    return np.array([
        [C, S],
        [-S, C]
    ])


def mkgrid(xspc, yspc, N, u, v, theta):
    mg = (np.mgrid[:N, :N].reshape(2, -1).T - [(N-1)/2, (N-1)/2]) * [xspc, yspc] - [u, v]
    mg = np.dot(Rmat(theta), mg.T)
    mg = np.vstack([mg, np.ones((1, mg.shape[1]))])
    mg = np.dot(cv2.Rodrigues(np.array([0, (-22.)*np.pi/180., 0]))[0], mg)
    mg = mg[:, mg[2] > 0]
    mg /= mg[2]
    return cv2.fisheye.distortPoints(mg.T[None, :, :2].astype(np.float32), K, dist)


def cost(xy, u, v, theta):
    N = xy.shape[1]
    x = xy[0]
    y = xy[1]
    S = np.sin(theta)
    C = np.cos(theta)
    dRx = x*S - C*y
    dRy = x*C + S*y
    dx = moddist(x*C + y*S - u, X_GRID)
    dy = moddist(-x*S + y*C - v, Y_GRID)
    S2 = np.sum(dRx)
    S3 = np.sum(dRy)
    JTJ = np.array([[N, 0, S2], [0, N, S3], [S2, S3, np.sum(x**2 + y**2)]])
    JTr = np.array([-np.sum(dx), -np.sum(dy), -np.sum(dx*dRx + dy*dRy)])
    return 0.5*np.sum(dx**2 + dy**2), -np.linalg.solve(JTJ + np.eye(3), JTr)


def render_floor(X, floorpx, fpts, mapsz=360, Z=5):
    ''' i've completely forgotten what all those parameters mean ... '''
    floormapbgr = np.zeros((mapsz, mapsz, 3))
    floormapN = np.ones((mapsz, mapsz))

    # FIXME: do it all in one bincount
    for k in range(len(X)):
        b = X[k]
        p = Z*(np.dot(Rmat(b[2]), fpts * FLOOR_HEIGHT).T + CEIL_HEIGHT * b[:2])
        mask2 = (p[:, 0] >= 0) & (p[:, 1] >= 0) & (p[:, 0] < mapsz-1) & (p[:, 1] < mapsz-1)
        p = p[mask2]
        pi = p.astype(np.int)
        idxs = pi[:, 1] * mapsz + pi[:, 0]

        floormapN[:] += np.bincount(idxs, np.ones(len(idxs)), mapsz*mapsz).reshape((-1, mapsz))
        for i in range(3):
            floormapbgr[:, :, i] += np.bincount(idxs, floorpx[k][mask2, i], mapsz*mapsz).reshape((-1, mapsz))

    return (floormapbgr / floormapN[:, :, None]).astype(np.uint8)
