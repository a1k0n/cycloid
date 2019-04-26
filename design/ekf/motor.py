import numpy as np
import sympy as sp
import codegen
import os


sp.init_printing()

# Define our model's state variables:

(v, delta, k1, k2, k3, srv_a, srv_b, srv_r) = sp.symbols("v delta k1 k2 k3 srv_a srv_b srv_r", real=True)

# dt, steering input, motor input
(Delta_t,  # time between prediction updates
 u_delta,  # control input for steering
 u_M       # control input for motor (assumed PWM controlled brushed DC)
 ) = sp.symbols("Delta_t u_delta u_M", real=True)


# state vector x is all of the above
X = sp.Matrix([v, delta, k1, k2, k3, srv_a, srv_b, srv_r])

print("state variables:")
sp.pprint(X.T)

ekfgen = codegen.EKFGen(X)

# Define a default initial state and covariance
x0 = np.float32([
    # v, delta,
    0, 0,
    # k1, k2, k3,
    1, 0.05, 0.03,
    # srv_a, srv_b, srv_r
    1, 0, 1,
])

P0 = np.float32([
    # v  - assume we start stationary
    1, 1,
    # k1, k2, k3
    1, 0.1, 0.1,
    # srv_a, srv_b, srv_r
    1, 1, 1,
])**2

try:
    os.mkdir("out_cc")
except:
    pass
try:
    os.mkdir("out_py")
except:
    pass
ekfgen.open("out_cc", "out_py", sp.Matrix(x0), sp.Matrix(P0))

u_DC = sp.Abs(u_M)
u_V = sp.Heaviside(u_M)  # 1 if u_M > 0, else 0
dv = Delta_t*(u_V*u_DC*k1 - u_DC*v*k2 - k3*v)
ddelta = Delta_t * srv_r * (srv_a*u_delta + srv_b - delta)

#ddelta = sp.Min(Delta_t * srv_r, sp.Abs(srv_a * u_delta + srv_b - delta)) * sp.sign(
#    srv_a * u_delta + srv_b - delta)

# The kinematics equations are based on a curvilinear unicycle model
# difficult to illustrate in a comment here, but I will eventually
# make an effort.
f = sp.Matrix([
    v + dv,
    delta + ddelta,
    k1, k2, k3,
    srv_a, srv_b, srv_r,
])

print("state transition: x +=")
sp.pprint(f - X)

# Our prediction error AKA process noise is kinda seat of the pants, but tuned
# on real runs by maximizing the subsequent measurement likelihood:
Q = sp.Matrix([
    # v, delta,
    0.5, 1e-5,
    # k1 k2 k3
    0.003, 1e-6, 1e-6,
    # srv_a, srv_b, srv_r
    0, 0, 0,
])

# Generate the prediction code:
ekfgen.generate_predict(f, sp.Matrix([u_M, u_delta]), Q, Delta_t)

# Now define the measurement models:

# delta is backwards from yaw rate, so negative here
h_imu = sp.Matrix([v * delta])
g_z = sp.symbols("g_z")
h_gyro = sp.Matrix([g_z])
R_gyro = sp.Matrix([1*v+1e-3])
ekfgen.generate_measurement(
    "IMU", h_imu, h_gyro, h_gyro, R_gyro)


# generate measurement for encoders
dsdt, wperiod = sp.symbols("dsdt wperiod")
h_z_encoders = sp.Matrix([dsdt, wperiod])
h_x_encoders = sp.Matrix([
    v, v
])
# measured std.dev
R_encoders = sp.Matrix([3.7, 3.5])

ekfgen.generate_measurement(
    "encoders", h_x_encoders,
    h_z_encoders, h_z_encoders, R_encoders)

ekfgen.close()
