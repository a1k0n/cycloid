import numpy as np
import sys
import cv2

camera_matrix = np.load("camera_matrix.npy")
dist_coeffs = np.load("dist_coeffs.npy")

new_camera_matrix = np.array([
    [-155, 0, 640],
    [0, 155, 960],
    [0, 0, 1]], np.float32)


def cal(jpg):
    # anglex, anglez = -0.037, -0.009
    anglex, anglez = -0.119, -0.0096
    while True:
        Rdown = cv2.Rodrigues(np.float32([-np.pi/2 + anglex, 0, anglez]))[0]
        udm1, udm2 = cv2.fisheye.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, Rdown, new_camera_matrix, (1280, 960),
            cv2.CV_16SC2)

        jpg_undist = cv2.remap(jpg, udm1, udm2, cv2.INTER_LINEAR)
        cv2.imshow('img', jpg_undist)
        print anglex, anglez
        while True:
            k = cv2.waitKey()
            if k == ord('q'):
                return Rdown
            if k == ord('a'):
                anglex -= 0.01
                break
            if k == ord('d'):
                anglex += 0.01
                break
            if k == ord('z'):
                anglex -= 0.001
                break
            if k == ord('c'):
                anglex += 0.001
                break
            if k == ord('w'):
                anglez -= 0.001
                break
            if k == ord('s'):
                anglez += 0.001
                break
            print k

if __name__ == '__main__':
    jpg = cv2.imread(sys.argv[1])
    Rdown = cal(jpg)

    # save out the camera transformation matrix so we can do the same
    # reprojection later
    np.save("Rdown", Rdown, False)

    # now we want to make a bunch of 25mm hisogram buckets and assign each
    # pixel into it from the 320x240 source
    camera_matrix_qvga = np.array([
        [-155/2, 0, 160],
        [0, 155/2, 240],
        [0, 0, 1]], np.float32)
    # so let's first map every pixel onto the physical plane using new_camera_matrix

    cv2.destroyAllWindows()
    cv2.waitKey(1)
