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
 srv_a, srv_b, srv_r,  # servo response model; delta -> srv_a * control + srv_b
                       # at rate srv_r
 srvfb_a, srvfb_b,  # servo feedback measurement = srvfb_a * delta + srvfb_b
 o_g  # gyroscope offset; gyro measures v * delta + o_g
 ) = sp.symbols("ml_1, ml_2, ml_3, srv_a, srv_b, srv_r, srvfb_a, srvfb_b, o_g",
                real=True)

# dt, steering input, motor input
(Delta_t,  # time between prediction updates
 u_delta,  # control input for steering
 u_M       # control input for motor (assumed PWM controlled brushed DC)
 ) = sp.symbols("Delta_t u_delta u_M", real=True)


# state vector x is all of the above
X = sp.Matrix([v, delta, y_e, psi_e, kappa,
               ml_1, ml_2, ml_3,
               srv_a, srv_b, srv_r, srvfb_a, srvfb_b, o_g])

print "state variables:"
sp.pprint(X.T)

ekfgen = codegen.EKFGen(X)

# [ 16488.1912919    3101.96113328 -38375.6189576 ]
# [ 16209.12647275   3192.14009973 -38327.66821225]
# [ 15849.81135393   3161.38822721 -38329.55155154]


# Define a default initial state and covariance
x0 = np.float32([
    # v, delta, y_e, psi_e, kappa
    0, 0, 0, 0, 0,
    # ml_1 (log m/s^2)
    3,
    # ml_2 (log 1/s back-EMF)
    0.77,
    # ml_3 (log 1/s friction)
    -0.7,
    # srv_a, srv_b, srv_r,
    -1.4, 0.2, 3.8,
    # srvfb_a, srvfb_b
    -35, 125,
    # o_g
    0])

P0 = np.float32([
    # v, delta, y_e, psi_e, kappa
    # assume we start stationary
    2., 0.1, 2, 1, 0.4,
    # ml_1, ml_2, ml_3
    0.2, 0.2, 0.2,
    # srv_a, srv_b, srv_r
    0.5, 0.5, 0.5,
    # srvfb_a, srvfb_b
    100, 100,
    # o_g
    1])**2

try:
    os.mkdir("out_cc")
except:
    pass
try:
    os.mkdir("out_py")
except:
    pass
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
#   k1 * u_V * u_DC - k2 * u_DC * v - k3*(static?) - k4
# where u_V is the control voltage signal (assumed 1 or 0) and u_DC is the
# duty cycle control input (from 0 to 1) and v is the current velocity.
# The ESC takes a positive or negative u_M control input which is transformed
# to u_DC and u_V here first.

# since k1, k2, k3, and k4 are scale constants, we keep them as logarithms in
# the model. That way they can never go negative, and we avoid huge derivatives
# when the relative scales are very different. This can, however, blow up to
# huge values if we're not carefully managing measurement and process noise.
# units:
# k1: m/s^2 / V  (acceleration per volt)
# k2: 1/s  (EMF decay time constant)
# k3: m/s^2  (coulomb friction, minimum torque to get moving)
# k4: m/s^2  (dynamic friction)
k1, k2, k3 = sp.exp(ml_1), sp.exp(ml_2), sp.exp(ml_3)

u_DC = sp.Abs(u_M)
u_V = sp.Heaviside(u_M)  # 1 if u_M > 0, else 0
dv = Delta_t*(u_V * u_DC * k1 - u_DC * v * k2 - v * k3)
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
    y_e - Delta_t * av * sp.sin(psi_e),
    psi_e + Delta_t * av * (-delta + kappa * sp.cos(psi_e) / (1 - kappa * y_e)),
    kappa,
    ml_1,
    ml_2,
    ml_3,
    srv_a,
    srv_b,
    srv_r,
    srvfb_a,
    srvfb_b,
    o_g
])

print "state transition: x +="
sp.pprint(f - X)


# Our prediction error AKA process noise is kinda seat of the pants, but tuned
# on real runs by maximizing the subsequent measurement likelihood:
Q = sp.Matrix([
    # v, delta, y_e, psi_e, kappa
    0.7, 0.7, 0.1*v + 1e-3, 0.15*v + 1e-3, 0.75*v + 1e-3,
    # ml_1, ml_2, ml_3
    0.1, 0.1, 0.1,
    # srv_a, srv_b, srv_r
    0, 0, 0,
    # srvfb_a, srvfb_b
    0, 0,
    # o_g
    1e-3])

# Generate the prediction code:
ekfgen.generate_predict(f, sp.Matrix([u_M, u_delta]), Q, Delta_t)


# Now define the measurement models:

# First we measure the road centerline's position, angle, and curvature with
# our camera / image processing pipeline. The result of that is a quadratic
# regression equation ax^2 + bx + c, and a quadratic fit covariance Rk also
# comes from our image processing pipeline.

def centerline_derivation():
    a, b, c, yc, t = sp.symbols("a b c y_c t", real=True)
    z_k = sp.Matrix([a, b, c, yc])

    # y_c is the center of the original datapoints, where our regression should
    # have the least amount of error. we will measure the centerline curvature
    # (kappa) and angle (psi_e) at this point, and then compute y_e as our
    # perpendicular distance to that line.

    # the regression line is x = a y^2 + b y + c
    xc = a*yc**2 + b*yc + c
    dx = sp.diff(xc, yc)
    dxx = sp.diff(dx, yc)
    kappa_est = dxx / ((dx**2 + 1)**(1.5))  # curvature at yc

    pc = sp.Matrix([xc, yc])  # regression center on curve
    N = sp.Matrix([-1, dx])  # N is a vector normal to the curve
    Nnorm = sp.sqrt((N.T * N)[0])  # length of normal

    # if curvature is low, assume we have a straight line; project our
    # regression centerpoint onto the unit normal vector to determine distance
    # to centerline, and tan(psi_e) = dx/dy = dx/1
    ye_linear_est = sp.simplify((N.T * pc)[0] / Nnorm)
    tanpsi_linear_est = dx

    # if curvature is nonzero, we're tracing a circle:
    # find the center of curvature by projecting 1/kappa meters along the unit
    # normal

    # curve_center = sp.simplify(pc + N / kappa_est)
    # print 'curve_center', curve_center
    # and then find the closest point on the circle to the car's CG (the
    # origin) by projecting back
    # curve_normal = sp.simplify(-curve_center /
    #                            sp.sqrt((curve_center.T * curve_center)[0]))
    # curve_refpoint = curve_center + curve_normal / kappa_est
    # ye_circular_est = (curve_refpoint.T * curve_normal)[0]
    # tanpsi_circular_est = sp.simplify(curve_center[0] / curve_center[1])

    # this is a (maybe bad) approximation, as the center point isn't
    # necessarily the correct point on the circular curve our model assumes,
    # but the math for circular curves isn't numerically stable when curvature
    # is close to zero. I'm hoping this works well enough as an approximation.

    h_x = sp.Matrix([y_e, psi_e, kappa])
    h_z = sp.Matrix([
        ye_linear_est,
        sp.atan(tanpsi_linear_est),
        # ye_circular_est,
        # sp.Piecewise((ye_linear_est, sp.Abs(kappa_est) < 1e-2),
        #             (ye_circular_est, True)),
        # tanpsi_circular_est,
        # sp.Piecewise((tanpsi_linear_est, sp.Abs(kappa_est) < 1e-2),
        #             (tanpsi_circular_est, True)),
        kappa_est
    ])

    return h_x, h_z, z_k

h_x_centerline, h_z_centerline, z_k_centerline = centerline_derivation()

ekfgen.generate_measurement(
    "centerline", h_x_centerline, h_z_centerline,
    z_k_centerline, sp.symbols("R_k"))

# delta is backwards from yaw rate, so negative here
h_imu = sp.Matrix([-v * delta + o_g])
h_imu

g_z = sp.symbols("g_z")
h_gyro = sp.Matrix([g_z])
R_gyro = sp.Matrix([0.1])
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
R_encoders = sp.Matrix([120, 7])

ekfgen.generate_measurement(
    "encoders", h_x_encoders,
    h_z_encoders, h_z_encoders, R_encoders)

ekfgen.close()
