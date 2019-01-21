# reproject an image to birdseye view containing fiducial markers (in this case, apriltags)

import cv2
import numpy as np

# foam board markers
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
side = 8.0*142.2/7.0 / 10.0  # size of markers, in cm
fidu_height = 101 / 10.0  # height from center of marker to floor

#side = 4.64
#aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

parameters = cv2.aruco.DetectorParameters_create()
#parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
#parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
#parameters.cornerRefinementWinSize = 9
# parameters.minDistanceToBorder = 1
parameters.minMarkerPerimeterRate = 0.003
#parameters.minCornerDistanceRate = 0.01
#parameters.maxErroneousBitsInBorderRate = 0.5
parameters.minMarkerDistanceRate = 0.0005

# intrinsic camera matrix and distortion from earlier calibration run
K = np.float32(
[[3.26903350e+03, 0.00000000e+00, 1.97581062e+03],
 [0.00000000e+00, 3.27551484e+03, 1.50907522e+03],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
D = np.float32([0.1038389, 0, 0.00073556, -0.00040451, 0])


def reproject(img, centerid=None, xshift=0, yshift=0, scale=1, size=1000, show=False):
    # first step: find the AprilTag markers (using the ArUco library)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    ids = ids.reshape(-1)
    print 'detected', len(ids), 'markers:', list(ids)
    idx = None

    if centerid is not None:
        idx = list(ids).index(centerid)

    rvecs, tvecs, _objpts = cv2.aruco.estimatePoseSingleMarkers(
        corners, side, K, D)

    if show:
        im2 = img[::5, ::5].copy()
        K2 = K.copy()
        K2[:2] /= 5
        for i in range(len(tvecs)):
            cv2.aruco.drawAxis(im2, K2, D, rvecs[i], tvecs[i], side)
        cv2.imwrite("preview.png", im2)
        cv2.imshow("axes", im2)
        cv2.waitKey()
        cv2.destroyAllWindows()
        cv2.waitKey(1)

    # compute relative rotation to bird's eye view
    R2 = np.eye(3)
    if idx is not None:
        # use initial orientation from selected center marker
        R2 = cv2.Rodrigues(rvecs[idx])[0].T
    if len(ids) < 4:
        # use average z vector as ground plane reference if we don't have
        # enough markers
        # wait, we're going to use the -y vector for upright markers
        z = -np.mean([cv2.Rodrigues(r)[0][:, 1] for r in rvecs[:, 0, ]], axis=0)
        z /= np.linalg.norm(z)
        R2[2] = z
    else:
        # compute the best-fit plane from the marker centers
        # best z vector is the eigenvector corresponding to the smallest
        # eigenvalue of the marker center matrix with centroid removed
        R1 = np.linalg.svd((tvecs[:, 0, :] - np.mean(tvecs[:, 0, :], axis=0)).T)[0]
        R2[2] = R1[:, 2]

    # orthonormalize
    R2[1] = np.cross(R2[0], R2[2])
    R2[1] /= np.linalg.norm(R2[1])
    R2[0] = np.cross(R2[2], R2[1])
    R2[0] /= np.linalg.norm(R2[0])

    T = np.eye(3)
    if idx is not None:
        c = np.dot(R2, tvecs[idx, 0])
        c /= c[2]
        T = np.array([
            [1, 0, -c[0]],
            [0, 1, -c[1]],
            [0, 0, 1]
        ])

    # get mean z-distance from camera for all markers
    # we could adjust this with args for upright fiducials
    dist = np.mean(np.dot(R2, tvecs[:, 0, :].T)[2]) - fidu_height

    # desired scale is 1cm/pixel
    # x' = f*x/dist; dx'/dx = 1/scale = f/dist; f = dist/scale
    M2 = np.array([
        [-dist/scale, 0, size/2 - xshift],
        [0, dist/scale, size/2 - yshift],
        [0, 0, 1]
    ])

    MM = np.dot(np.dot(M2, T), np.dot(R2, np.linalg.inv(K)))

    return cv2.warpPerspective(img, MM, (size, size))


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='Reproject image containing AprilTags to birdseye view')
    parser.add_argument('--img', required=True, help="image file")
    parser.add_argument('--out', help="output file")
    parser.add_argument('--centerid', type=int, help="marker id to center image on (optional)")
    parser.add_argument('--scale', type=float, default=1.0, help="pixel scale (cm/pixel)")
    parser.add_argument('--size', type=int, default=1000, help="output image size")
    parser.add_argument('--show', action='store_true', help="show marker detections first")
    parser.add_argument('-x', type=float, default=0.0, help="x shift")
    parser.add_argument('-y', type=float, default=0.0, help="y shift")

    args = parser.parse_args()

    im = cv2.imread(args.img)
    out = reproject(im, args.centerid, args.x, args.y, args.scale, args.size, args.show)
    if args.out is None:
        cv2.imshow("output", out)
        cv2.waitKey()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
    else:
        cv2.imwrite(args.out, out)
