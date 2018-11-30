import numpy as np
import cv2
from params import vpy, turn_slope, threshold, bandheight


kernel = np.float32([-1, -1, -1, 2, 2, 2, -1, -1, -1])


camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
camera_matrix[:2] /= 4.  # for 640x480


def classify(yuv420img, gyroz):
    # get V channel from YUV
    imgv = np.copy(yuv420img[600:].reshape((-1, 320)))
    slopevp = np.int32(vpy - np.linspace(-1, 1, 640)*turn_slope*gyroz)
    slopevp2 = np.int32(vpy/2 - np.linspace(-1, 1, 320)*turn_slope*gyroz/2)
    # convolutional filter which activates on cones on scanlines near the
    # vanishing point
    A = np.zeros(320, np.float32)
    for i in range(bandheight/2):  # add up 4 scanlines of filters
        a = np.convolve(imgv[slopevp2 + i, np.arange(320)],
                        kernel, mode='same')
        a[:5] = 0
        a[-5:] = 0
        A += a

    A = A > threshold  # classify
    origA = A.copy()
    for i in range(5):
        A[1:] |= A[:-1]
        A[:-1] |= A[1:]

    # find centers of each connected range
    l = None
    conecenters = []
    conewidths = []
    for i in range(len(A)):
        if not A[i] and l is not None:
            center = (i-1 + l)
            width = i-1 - l
            conecenters.append([center, slopevp[center]+bandheight])
            conewidths.append(width)
            l = None
        elif A[i] and l is None:
            l = i

    origcenters = conecenters
    if len(conecenters) > 0:
        origcenters = conecenters
        conecenters = cv2.fisheye.undistortPoints(np.array([conecenters], np.float32), camera_matrix, dist_coeffs)[0]

    return conecenters, origcenters, origA


if __name__ == '__main__':
    # read a replay recording, classify all the cones, and store it in a pickle
    # for pftune.py
    pass
