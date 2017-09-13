import numpy as np
import cv2

import imgremap


ytop = imgremap.ytop


def generate_maps():
    camera_matrix = np.load("../camcal/camera_matrix.npy")
    dist_coeffs = np.load("../camcal/dist_coeffs.npy")
    Rdown = np.load("../camcal/Rdown.npy")

    print "loaded calibration from ../autorustler/camcal/"

    camera_matrix[:2] /= 8.

    camera_mm_scale = 153  # tunable parameter to scale 1mm/pixel
    pixel_scale_mm = 25  # but we use 25mm buckets to detect 50mm line position

    undistort_map = cv2.fisheye.undistortPoints(
        np.mgrid[:320, :240].T.astype(np.float32),
        camera_matrix, dist_coeffs)

    ytop = 100
    udplane = np.dot(np.concatenate(
        [undistort_map, np.ones((240, 320, 1))], axis=2), Rdown.T)[ytop:]
    # udplane is now in 25mm units
    udplane[:, :, :2] *= -camera_mm_scale / pixel_scale_mm
    # and perspective correct
    udplane[:, :, 0] /= udplane[:, :, 2]
    udplane[:, :, 1] /= udplane[:, :, 2]

    # so udplane[y, x, :] assigns camera space x, y to a 25mm grid pixel in
    # front of us

    # mask off pixels which are behind us (>50mm ahead of camera because of
    # bumper protrusion) or are too far away to clearly see we have about a
    # 1.5-meter radius
    udmask = (udplane[:, :, 1] > 2) & (
        (udplane[:, :, 0]**2 + udplane[:, :, 1]**2) < 60**2)

    udplane = np.int8(udplane.T * udmask.T).T

    uxrange = min(udplane[udmask, 0]), max(udplane[udmask, 0]) + 1
    uyrange = min(udplane[udmask, 1]), max(udplane[udmask, 1]) + 1
    x0, y0 = uxrange[0], uyrange[0]

    bucketcount = np.zeros((uyrange[1] - y0, uxrange[1] - x0))
    np.add.at(bucketcount,
              (udplane[udmask, 1] - y0, udplane[udmask, 0] - x0),
              1)

    # flood fill to get nearest pixel location for every point where
    # udmask == 0
    floodmap = np.arange(bucketcount.shape[0] * bucketcount.shape[1]).reshape(
        (-1, bucketcount.shape[1]))
    floodmap[bucketcount == 0] = -1
    while np.any(floodmap == -1):
        # propagate any value that isn't -1 onto neighbors of existing -1s
        # propagate right
        floodmap[:, 1:][floodmap[:, 1:] == -1] = \
            floodmap[:, :-1][floodmap[:, 1:] == -1]
        # left
        floodmap[:, :-1][floodmap[:, :-1] == -1] = \
            floodmap[:, 1:][floodmap[:, :-1] == -1]
        # propagate down
        floodmap[1:][floodmap[1:] == -1] = floodmap[:-1][floodmap[1:] == -1]
        # up
        floodmap[:-1][floodmap[:-1] == -1] = floodmap[1:][floodmap[:-1] == -1]

    np.savetxt("udplane.txt", udplane[:, :, :2].reshape(-1), fmt='%d', newline=',\n')
    np.savetxt("udmask.txt", udmask.reshape(-1), fmt='%d', newline=',\n')
    invbucketcount = np.copy(bucketcount)
    invbucketcount[bucketcount != 0] = 1.0 / bucketcount[bucketcount != 0]
    np.savetxt("bucketcount.txt", invbucketcount.reshape(-1), fmt='%f', newline=',\n')
    np.savetxt("floodmap.txt", floodmap.reshape(-1), fmt='%d', newline=',\n')

    floodmap = np.stack([
        floodmap / bucketcount.shape[1],
        floodmap % bucketcount.shape[1]], axis=2)[bucketcount == 0]

    np.save("udplane", udplane)
    np.save("udmask", udmask)
    np.save("bucketcount", invbucketcount)
    np.save("floodmap", floodmap)
    print "maps saved, output is %d x %d" % (bucketcount.shape[1], bucketcount.shape[0])
    print "uxrange", uxrange, "uyrange", uyrange, 'x0', x0, 'y0', y0


if __name__ == '__main__':
    generate_maps()
