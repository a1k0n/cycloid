import numpy as np
import sympy as sp
import codegen
import os


sp.init_printing()
try:
    os.mkdir("localize_cc")
    os.mkdir("localize_py")
except Exception:
    pass

# Define our model's state variables:
(x, y, theta) = sp.symbols("p_x p_y theta", real=True)

# dt, wheel encoder distance, yaw rate
(Delta_t, u_x, u_theta) = sp.symbols("Delta_t u_x u_theta")

X = sp.Matrix([x, y, theta])

ekfgen = codegen.EKFGen(X)

# initial state and covariance
x0 = np.float32([0, 0, 0])

# this depends on whether we are mapping or localizing
# if building the initial map then our covariance is 0 at initialization
# but assume we're placed somewhere "near" the start, pointing "mostly" forward for now
P0 = np.float32([0.1, 0.1, 0.1])**2

ekfgen.open("localize_cc", "localize_py", sp.Matrix(x0), sp.Matrix(P0))

# Prediction update
# "predictions" are done based on gyro and encoder measurements
theta1 = theta + Delta_t * u_theta/2
f = sp.Matrix([
    x + u_x * sp.cos(theta1),
    y + u_x * sp.sin(theta1),
    theta + Delta_t * u_theta
])

# Our prediction error AKA process noise
# gyro errors are small, and encoder measurements are large-ish along the
# forward direction and tiny in the lateral direction (usually!)
Qlong = 200*u_x  # longitudinal error
Qlat = 0.25*u_theta  # latitude error increases with yaw rate
Qgyro = 0.02
Q = sp.Matrix([
    [sp.cos(theta1)**2 * Qlong + sp.sin(theta1)**2 * Qlat,
     (Qlong-Qlat)*sp.sin(2*theta1)/2, 0],  # x
    [(Qlong-Qlat)*sp.sin(2*theta1)/2,
     sp.sin(theta1)**2 * Qlong + sp.cos(theta1)**2 * Qlat, 0],  # y
    [0, 0, Qgyro],
])

print "state transition: x +="
sp.pprint(f - X)

ekfgen.generate_predict(f, sp.Matrix([u_x, u_theta]), Q, Delta_t)


# Bearing measurement of landmark
lx, ly = sp.symbols("l_x l_y")
def bearing_measurement(lx, ly):
    s, c = sp.sin(theta), sp.cos(theta)
    R = sp.Matrix([[c, s], [-s, c]])
    lo = R * sp.Matrix([lx - x, ly - y])
    return sp.Matrix([sp.atan2(lo[1], lo[0])])


h_x_bearing = bearing_measurement(lx, ly)
l_px = sp.symbols("l_px")
h_z_bearing = sp.Matrix([l_px])
z_bearing = sp.Matrix([l_px, lx, ly])
R_bearing = sp.symbols("lm_R")
ekfgen.generate_measurement(
    "lm_bearing", h_x_bearing, h_z_bearing, z_bearing, R_bearing)

ekfgen.close()
