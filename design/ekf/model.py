import numpy as np
import sympy as sp
import codegen
import os


sp.init_printing()


# Define our model's state variables:

(v,      # velocity (m/s)
 delta,  # steering curvature (1/m) - inverse turning radius @ steering
         # position
 y_e,    # lateral distance from centerline; + means car is right of line
 psi_e,  # car's angle w.r.t. centerline; + means car is facing
         # counterclockwise from line
 kappa   # line curvature (1/m)
 ) = sp.symbols("v, delta, y_e, psi_e, kappa", real=True)

(ml_1, ml_2, ml_3,  # log of brushed DC motor model constants
 ml_4,              # log of static friction constant
 srv_a, srv_b, srv_r,  # servo response model; delta -> srv_a * control + srv_b
                       # at rate srv_r
 srvfb_a, srvfb_b,  # servo feedback measurement = srvfb_a * delta + srvfb_b
 o_g  # gyroscope offset; gyro measures v * delta + o_g
 ) = sp.symbols("ml_1, ml_2, ml_3, ml_4, srv_a, srv_b, srv_r, srvfb_a, srvfb_b, o_g",
                real=True)

# dt, steering input, motor input
(Delta_t,  # time between prediction updates
 u_delta,  # control input for steering
 u_M       # control input for motor (assumed PWM controlled brushed DC)
 ) = sp.symbols("Delta_t u_delta u_M", real=True)


# state vector x is all of the above
X = sp.Matrix([v, delta, y_e, psi_e, kappa,
               ml_1, ml_2, ml_3, ml_4,
               srv_a, srv_b, srv_r, srvfb_a, srvfb_b, o_g])

print "state variables:"
sp.pprint(X.T)

ekfgen = codegen.EKFGen(X)

# Define a default initial state and covariance
x0 = np.float32([
    # v, delta, y_e, psi_e, kappa
    0, 0, 0, 0, 0,
    # m1_ (log m/s^2) (overestimated for slow start)
    3.7,
    # ml_2, ml_3 (both log 1/s)
    1.6, 0.98,
    # ml_4 (log m/s^2 static frictional deceleration)
    1,
    # srv_a, srv_b, srv_r,
    0.85,   -0.14,   2.2,
    # srvfb_a, srvfb_b
    -180, 107.8,
    # o_g
    0])

P0 = np.float32([
    # v, delta, y_e, psi_e, kappa
    # assume we start stationary
    0.001, 0.1, 2, 1, 1,
    # ml_1, ml_2, ml_3, ml_4
    0.25, 0.25, 0.25, 0.25,
    # srv_a, srv_b, srv_r
    0.1, 0.1, 0.1,
    # srvfb_a, srvfb_b
    50, 10,
    # o_g
    1])**2

os.mkdir("out_cc")
os.mkdir("out_py")
ekfgen.open("out_cc", "out_py", sp.Matrix(x0), sp.Matrix(P0))

# The brushed DC motor model has three components:
# The electronic "speed controller" is really just a voltage source
# and a PWM-controlled electronic switch which either applies a voltage
# to the motor in pulses (when input control signal is positive),
# or shorts the motor out in pulses (when it is negative).

# The motor generates a back-EMF proportional to its velocity. This EMF
# is subtracted from the input voltage (which is either V+ when accelerating
# or 0 when braking), and while the PWM switch is closed, slows the motor
# down. The motor is also always slowed down by friction.

# The car's acceleration is proportional to the torque produced by the motor,
# so we fold the inertia into the system constants. The car's acceleration is
# thus:
#   k1 * u_V * u_DC - k2 * u_DC * v - k3 * v
# where u_V is the control voltage signal (assumed 1 or 0) and u_DC is the
# duty cycle control input (from 0 to 1) and v is the current velocity.
# The ESC takes a positive or negative u_M control input which is transformed
# to u_DC and u_V here first.

# since k1, k2, and k3 are scale constants, we keep them as logarithms in the
# model. That way they can never go negative, and we avoid huge derivatives when
# the relative scales are very different. This can, however, blow up to huge
# values if we're not carefully managing measurement and process noise.
# units:
# k1: m/s^2 / V  (acceleration per volt)
# k2: 1/s  (EMF decay time constant)
# k3: 1/s  (friction decay time constant)
# k4: m/s^2  (coulomb friction, minimum torque to get moving)
k1, k2, k3, k4 = sp.exp(ml_1), sp.exp(ml_2), sp.exp(ml_3), sp.exp(ml_4)

u_DC = sp.Abs(u_M)
u_V = sp.Heaviside(u_M)  # 1 if u_M > 0, else 0
# we also have a static friction coefficient which tries to make the velocity
# exactly 0, up to the friction limit
dv = Delta_t*(u_V * u_DC * k1 - u_DC * v * k2 - v * k3 - k4)
dv = sp.Max(dv, -v)  # velocity cannot go negative
av = v + dv / 2  # average velocity during the timestep


# The servo has its own control loop and position feedback built in, but
# we need to model how it relates to the car's actual rotation, so the
# "servo" constants here also encompass the steering geometry of the car.

# We assume here that the servo linearly slews to the desired position, with
# a certain ratio (srv_a) between control input and turning curvature (1/radius),
# a certain offset (srv_b) when the control signal is 0, and a linear slew
# rate srv_r.

# The math for this is kind of messy; the code would be simpler as some if
# statements, but this needs to be a differentiable function.

ddelta = sp.Min(Delta_t * srv_r, sp.Abs(srv_a * u_delta + srv_b - delta)) * sp.sign(
    srv_a * u_delta + srv_b - delta)

# The kinematics equations are based on a curvilinear unicycle model
# difficult to illustrate in a comment here, but I will eventually
# make an effort.
f = sp.Matrix([
    v + dv,
    delta + ddelta,
    y_e + Delta_t * av * sp.sin(psi_e),
    psi_e + Delta_t * av * (delta + kappa * sp.cos(psi_e) / (1 - kappa * y_e)),
    kappa,
    ml_1,
    ml_2,
    ml_3,
    ml_4,
    srv_a,
    srv_b,
    srv_r,
    srvfb_a,
    srvfb_b,
    o_g
])

print "state transition: x +="
sp.pprint(f - X)

# Our prediction error AKA process noise is kinda seat of the pants:
Q = sp.Matrix([
    # v, delta, y_e, psi_e, kappa
    4, 2, 1, 1, 10,
    # ml_1, ml_2, ml_3, ml_4
    1e-1, 1e-2, 1e-2, 1e-1,
    # srv_a, srv_b, srv_r
    1e-2, 1e-2, 1e-2,
    # srvfb_a, srvfb_b
    1e-3, 1e-5,
    # o_g
    1e-5])

# Generate the prediction code:
ekfgen.generate_predict(f, sp.Matrix([u_M, u_delta]), Q, Delta_t)


# Now define the measurement models:

# First we measure the road centerline's position, angle, and curvature with
# our camera / image processing pipeline. The result of that is a quadratic
# regression equation ax^2 + bx + c, and a quadratic fit covariance Rk also
# comes from our image processing pipeline.
a, b, c = sp.symbols("a b c", real=True)
z_k_centerline = sp.Matrix([a, b, c])

# These parameters correspond to our state in the following way:
# We measure an approximate y_e, psi_e, and kappa:
h_x_centerline = sp.Matrix([y_e, psi_e, kappa])
h_z_centerline = sp.Matrix([
    -c,
    sp.atan(b),
    2*a * (b**2 + 1)**-1.5
])

ekfgen.generate_measurement(
    "centerline", h_x_centerline, h_z_centerline,
    z_k_centerline, sp.symbols("R_k"))

# delta is backwards from yaw rate, so negative here
h_imu = sp.Matrix([-v * delta + o_g])
h_imu

g_z = sp.symbols("g_z")
h_gyro = sp.Matrix([g_z])
R_gyro = sp.Matrix([0.1])  # measured noise std.dev
ekfgen.generate_measurement(
    "IMU", h_imu, h_gyro, h_gyro, R_gyro)


# generate measurement for encoders
METERS_PER_ENCODER_TICK = np.pi * 0.101 / 20
dsdt, fb_delta = sp.symbols("dsdt fb_delta")
h_z_encoders = sp.Matrix([dsdt, fb_delta])
h_x_encoders = sp.Matrix([
    v / METERS_PER_ENCODER_TICK,
    srvfb_b + delta * srvfb_a
])
R_encoders = sp.Matrix([1, 1])

ekfgen.generate_measurement(
    "encoders", h_x_encoders,
    h_z_encoders, h_z_encoders, R_encoders)

ekfgen.close()
