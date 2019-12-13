from ceiltrack import CAM_TILT
from generate import rle
import cv2
import numpy as np
import struct
import sys


def undistort(fx, fy, cx, cy, k1, camtilt, camrot):
    ''' undistortPoints doesn't support points behind the image plane, but we
    can solve for them '''
    def solvetheta(thetad, k1):
        theta = thetad
        theta += (theta*(k1*theta**2 + 1) - thetad)/(-3*k1*theta**2 - 1)
        theta += (theta*(k1*theta**2 + 1) - thetad)/(-3*k1*theta**2 - 1)
        return theta

    mg = np.mgrid[:480, :640].transpose(0, 2, 1)
    u, v = (mg[1] - cx)/fx, (mg[0] - cy)/fy
    r = np.sqrt(u**2 + v**2)
    a, b = u/r, v/r
    theta = solvetheta(r, k1)
    t = 1.0 / np.tan(np.pi/2 - theta)

    # (a*t, b*t, 1) is the direction vector; we can then rotate with R
    # except the z-direction might be -1 depending on what theta is...

    Ct, St = np.cos(camtilt), np.sin(camtilt)
    Ry = np.array([
        [Ct, 0, St],
        [0, 1, 0],
        [-St, 0, Ct]
    ])
    Cr, Sr = np.cos(camrot), np.sin(camrot)
    Rz = np.array([
        [Cr, -Sr, 0],
        [Sr, Cr, 0],
        [0, 0, 1]
    ])
    R = np.dot(Ry, Rz)
    return np.dot(R, np.stack([a * np.abs(t), b * np.abs(t), np.sign(t)]).transpose(2, 0, 1))


def writefloorlut(pts, frontfloormask):
    # half-resolution points
    hpts = pts[:, ::2, ::2]
    hmask = frontfloormask[::2, ::2]
    yangle = np.round(np.arctan2(pts[1, frontfloormask], pts[0, frontfloormask]) * 256 / np.pi).astype(np.int8)
    uvangle = np.round(np.arctan2(hpts[1, hmask], hpts[0, hmask]) * 256 / np.pi).astype(np.int8)
    print(np.min(yangle), np.max(yangle), np.min(uvangle), np.max(uvangle))

    fname = "floorlut.bin"
    f = open(fname, "wb")
    # header:
    #  - uint16 image height
    #  - uint16 image width
    #  - uint32 number of pixels in mask (N)
    #  - uint32 rle-compressed mask size in bytes (M)
    # followed by
    # [uint16 skip, uint16 run] x (M/4)
    # [int8 angle] x N
    rlemask = rle(frontfloormask.reshape(-1)).astype(np.uint16)
    uvrlemask = rle(hmask.reshape(-1)).astype(np.uint16)
    h, w = frontfloormask.shape
    hlen = 2 + 2 + 4 + 4 + 4 + 4
    f.write(struct.pack("=4sIHHIIII", b'fmLU', hlen, h, w,
                        np.sum(frontfloormask), len(rlemask),
                        np.sum(hmask), len(uvrlemask)))
    f.write(rlemask.tobytes())
    f.write(yangle.astype(np.int8).tobytes())
    f.write(uvrlemask.tobytes())
    f.write(uvangle.astype(np.int8).tobytes())
    f.close()
    print("wrote", fname, 'mask', len(rlemask)*2, 'bytes; pts x', len(yangle), ' angle bytes')


def writefrontlut(K, dist):
    Knew = np.array([
        [-120., 0, 160],
        [0, 120., 71],
        [0, 0, 1.0]
    ])
    R = cv2.Rodrigues(np.array([0, CAM_TILT[1] - np.pi/2, 0]))[0]
    R = np.dot(np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]]), R)
    xmap, ymap = cv2.fisheye.initUndistortRectifyMap(
        K, dist, R, Knew, (320, 120), cv2.CV_32FC1)
    fname = "frontlut.bin"
    f = open(fname, "wb")
    hlen = 2 + 2
    f.write(struct.pack("=4sIHH", b'10.6', hlen, 320, 120))
    imap = np.stack([xmap, ymap]).transpose((1, 2, 0))
    print("shape:", imap.shape)
    print(np.max(imap[-1, :, 0]))
    f.write((64*imap).astype(np.uint16).tobytes())
    f.close()
    print("wrote frontlut.bin")


def main(fname):
    view = cv2.imread(fname)

    K = np.load("../camcal/camera_matrix.npy")
    dist = np.load("../camcal/dist_coeffs.npy")
    K[:2] /= 4.05
    fx, fy = np.diag(K)[:2]
    cx, cy = K[:2, 2]

    writefrontlut(K, dist)

    pts = undistort(fx, fy, cx, cy, dist[0], CAM_TILT[1], 0)

    r = np.sum((pts[:2] / pts[2])**2, axis=0)
    prelim_mask = ((pts[2] < 0) & (pts[0] > 0)) & (r > 4**2)
    im = prelim_mask[:, :, None] * view
    #im[~prelim_mask, :] = 255
    cv2.imwrite("premask.png", im)

    # if the prelim mask is good enough, then we don't need to do anything else
    # so go ahead and write out floorlut.bin
    writefloorlut(pts, prelim_mask)


if __name__ == '__main__':
    main(sys.argv[1])
