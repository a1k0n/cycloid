# bundle adjustment frontend -- extract ceiling lights and cones
# from cycloid recording file (video + wheel odometry + gyro)
import numpy as np
import cv2

# parameters:
MIN_CEIL_LIGHT_DIST = 2.0  # (meters)
MIN_CONE_ANGLE = np.pi/8
CAM_TILT = np.array([0, 22. * np.pi / 180., 0])
WHEELTICK_SCALE = 0.066

# initial guesses for learned parameters
# ceiling height measured 8'5", camera is about 4" off ground
CEILING_HEIGHT_GUESS = 2.4  # (meters)
CEILING_TILE_GRID = 0.610  # (meters) -- measured 24"
# ceiling lights are in a 5x6-tile pattern, each tile is 24"
# ceiling light centers should be 10' (3.05m) apart along the short axis
#  and 12' apart (3.66m) along the long axis


def undistort():
    K = np.load("../../tools/camcal/camera_matrix.npy")
    dist = np.load("../../tools/camcal/dist_coeffs.npy")
    K[:2] /= 4.05
    fx, fy = np.diag(K)[:2]
    cx, cy = K[:2, 2]
    ''' undistortPoints doesn't support points behind the image plane, but we can solve for them '''
    def solvetheta(thetad, k1):
        theta = thetad
        theta += (theta*(k1*theta**2 + 1) - thetad)/(-3*k1*theta**2 - 1)
        theta += (theta*(k1*theta**2 + 1) - thetad)/(-3*k1*theta**2 - 1)
        return theta

    mg = np.mgrid[:480, :640].transpose(0, 2, 1)
    u, v = (mg[1] - cx)/fx, (mg[0] - cy)/fy
    r = np.sqrt(u**2 + v**2)
    a, b = u/r, -v/r
    theta = solvetheta(r, dist[0])
    t = 1.0 / np.tan(theta - np.pi/2)
    # (a*t, b*t, 1) is the direction vector; we can then rotate with R
    # except the z-direction might be -1 depending on what theta is...
    R = cv2.Rodrigues(CAM_TILT)[0]
    return np.dot(R, np.stack([a * t, b * t, np.ones(a.shape)]).transpose(2, 0, 1))


def genlut():
    K = np.load("../../tools/camcal/cl20200307/camera_matrix.npy")
    dist = np.load("../../tools/camcal/cl20200307/dist_coeffs.npy")
    print("cl2020307")
    K[:2] /= 4.05
    fx, fy = np.diag(K)[:2]
    cx, cy = K[:2, 2]
    uv = np.mgrid[:480, :640][[1, 0]].transpose(1, 2, 0).astype(np.float32)
    # ceilmask = ((uv[:, :, 1] - cy)**2 + (uv[:, :, 0] - cx + 60)**2) < (np.pi/2.4 * fx)**2
    R = cv2.Rodrigues(CAM_TILT)[0]
    origpts = cv2.fisheye.undistortPoints(uv, K=K, D=dist)
    pts = np.stack([origpts[:, :, 0], origpts[:, :, 1], np.ones((480, 640))])
    return np.dot(R, pts.transpose(1, 0, 2)), origpts
