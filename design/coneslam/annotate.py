import cv2
import numpy as np
import params


METERS_PER_ENCODER_TICK = params.WHEEL_TICK_LENGTH


def draw_steering(bgr, steering, servo, center=(320, 420)):
    # make steering wheel, lower center
    #servo = 128*(servo - 125)/70.0
    servo = steering
    # sdeg = steering  # just 1:1 i guess?
    sdeg = params.STEER_DIRECTION*servo  # just 1:1 i guess?
    srad = sdeg * np.pi / 180.0
    S, C = 16*30*np.sin(srad), 16*30*np.cos(srad)
    cv2.circle(bgr, center, 30, (255, 255, 255), 1, cv2.LINE_AA)
    scenter = (center[0]*16, center[1]*16)
    cv2.line(bgr, (int(scenter[0] - C), int(scenter[1] + S)),
             (int(scenter[0] + C), int(scenter[1] - S)),
             (255, 255, 255), 1, cv2.LINE_AA, 4)
    cv2.ellipse(bgr, center, (30, 30), 0, -90, -90 + steering,
                (255, 180, 180), 5, cv2.LINE_AA)
    cv2.ellipse(bgr, center, (30, 30), 0, -90, -90 + servo,
                (0, 180, 255), 2, cv2.LINE_AA)


last_ts = None
last_wheels = None


def draw_speed(bgr, tstamp, wheels, periods, center=(40, 420), radius=30):
    # draw a little spedometer in the lower left
    # just draw the needle for each period now
    global last_ts, last_wheels

    av = np.mean(periods[:params.NUM_ENCODERS])
    if av != 0:
        av = METERS_PER_ENCODER_TICK * 1e6 / av
    # cv2.putText(bgr, "%0.1f %0.1f %0.1f %0.1f m/s" % tuple(v), (10, 470),
    #             cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1,
    #             cv2.LINE_AA)

    if last_ts is None:
        last_ts = tstamp
        last_wheels = wheels
        return

    dw = wheels - last_wheels
    if np.all(dw == 0):
        last_ts = tstamp
        last_wheels = wheels
        return
    # vv = METERS_PER_ENCODER_TICK * np.float32(dw) / (tstamp - last_ts)
    # av = 0.5 * np.mean(v[dw != 0] + vv[dw != 0])

    mph = 2.23694 * av

    # draw ticks
    for i in range(13):
        phi = (i - 6) * 0.4
        C, S = radius * np.cos(phi), radius * np.sin(phi)
        cv2.line(bgr, (int(center[0] + S), int(center[1] - C)),
                 (int(center[0] + 0.8*S), int(center[1] - 0.8*C)),
                 (255, 255, 255), 1, cv2.LINE_AA)

    phi = (mph - 6) * 0.4
    C, S = radius * np.cos(phi), radius * np.sin(phi)
    cv2.line(bgr, (int(center[0] + S), int(center[1] - C)),
             (int(center[0]), int(center[1])),
             (180, 255, 180), 2, cv2.LINE_AA)

    cv2.putText(bgr, "%0.1f mph" % (mph), (30, 460),
                cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1,
                cv2.LINE_AA)
    last_ts = tstamp
    last_wheels = wheels


def draw_throttle(img, throttle, center=(320, 470)):
    cv2.line(img, center, (center[0] + throttle, center[1]),
             throttle > 0 and (0, 255, 0) or (0, 95, 255), 5)


def draw_accelerometer(bgr, accel, gyro, center=(470, 470)):
    cv2.circle(bgr, center, 30, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.ellipse(bgr, center, (30, 30), 0, -90, -90 - 180*gyro[2] / np.pi,
                (100, 255, 180), 3, cv2.LINE_AA)
    cv2.line(bgr, center, (int(center[0] - accel[1]*30),
                           int(center[1] + accel[0]*30)),
             (100, 255, 100), 2, cv2.LINE_AA)
