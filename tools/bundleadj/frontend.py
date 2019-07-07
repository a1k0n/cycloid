# bundle adjustment frontend -- extract ceiling lights and cones
# from cycloid recording file (video + wheel odometry + gyro)
import numpy as np

# parameters:
MIN_CEIL_LIGHT_DIST = 1.0  # (meters)
MIN_CONE_ANGLE = np.pi/8
CAMERA_TILT = 22 * np.pi / 180.
WHEELTICK_SCALE = 0.066

# initial guesses for learned parameters
CEILING_HEIGHT_GUESS = 3.5  # (meters)


def genlut():
    K = np.load("../../tools/camcal/old/camera_matrix.npy")
    dist = np.load("../../tools/camcal/old/dist_coeffs.npy")
    K[:2] /= 4.05  # full res -> 640x480
    fx, fy = np.diag(K)[:2]
    cx, cy = K[:2, 2]
    uv = np.mgrid[:480, :640][[1, 0]].transpose(1, 2, 0).astype(np.float32)
    ceilmask = ((uv[:, :, 1] - cy)**2 + (uv[:, :, 0] - cx)**2) < (np.pi/2.2 * fx)**2
    pts = cv2.fisheye.undistortPoints(uv[None, ceilmask], K, dist)

