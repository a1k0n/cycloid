# coefficients for servo feedback -> front wheel angle:
# radians = (servo-126.5) / 121.3


def wheel_angle(servo_fb):
    return (servo_fb - 126.5) / 121.3


def servo_target(control_input):
    m, b = 0.3876, 119.5
    return m*control_input + b


def motor_constants():
    ''' k1, k2, k3, control offset (in 0..1.0 PWM space) '''
    return 2.58, 0.0931, 0.218, 0.103
