
from ceiltrack import CAM_TILT
from floormask import undistort, writefloorlut
import numpy as np
import cv2
import sys


def main(fname):
    mask = cv2.imread(fname)[:, :, 0] != 0
    cv2.imwrite("floormask.png", 255 * mask)

    K = np.load("../camcal/camera_matrix.npy")
    dist = np.load("../camcal/dist_coeffs.npy")
    K[:2] /= 4.05
    fx, fy = np.diag(K)[:2]
    cx, cy = K[:2, 2]

    pts = undistort(fx, fy, cx, cy, dist[0], CAM_TILT[1], 0)

    # if the prelim mask is good enough, then we don't need to do anything else
    # so go ahead and write out floorlut.bin
    writefloorlut(pts, mask)


if __name__ == '__main__':
    main(sys.argv[1])
