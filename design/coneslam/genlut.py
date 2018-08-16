import numpy as np
import cv2

camera_matrix = np.load("../../tools/camcal/camera_matrix.npy")
dist_coeffs = np.load("../../tools/camcal/dist_coeffs.npy")
camera_matrix[:2] /= 4.  # for 640x480

vpy = 207  # y vanishing point on a 640x480 image
turn_slope = 15
width = 8

pts = np.mgrid[:640, :turn_slope*4+width].T + np.array([0, vpy])
xyu = cv2.fisheye.undistortPoints(
    np.array(pts, np.float32), camera_matrix, dist_coeffs)

# we only have 240 lines of vertical resolution, so skip every other line
# we have 640 horizontal resolution only because we're taking the average of a
# row of pixels and that can end up between pixels
data = np.array(np.arctan(xyu[::2, :, 0]), np.float32)

np.save("lut.npy", data)

print "const int conedetect_vpy = %d;" % vpy
print "const float conedetect_turn_slope = %f;" % (turn_slope / 2.0)
print "const int conedetect_y_offset = %d;" % (turn_slope - vpy/2)
print "const int conedetect_width= %d;" % (width / 2)
d = data.reshape(-1)
print "\nconst float conedetect_LUT[%d] = {" % len(d)
for i, v in enumerate(d):
    print "  %f%s" % (v, "" if i == len(d) - 1 else ",")
print "};"
