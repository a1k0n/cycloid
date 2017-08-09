import numpy as np
import ekf


def ekf_test():
    x, P = ekf.initial_state()

    for u in np.linspace(0, 0.2, 10):
        x, P = ekf.predict(x, P, 0.0625, u, 0)
        print " negligible accel", u, x

    x, P = ekf.predict(x, P, 0.0625, 1, 0)
    x, P = ekf.predict(x, P, 0.0625, 1, 0)
    x, P = ekf.predict(x, P, 0.0625, 1, 0)
    print 'after 3 accel', x
    x, P = ekf.predict(x, P, 0.0625, -1, 0)
    x, P = ekf.predict(x, P, 0.0625, -1, 0)
    print 'after 2 brake', x
    for i in range(100):
        x, P = ekf.predict(x, P, 0.0625, -1, 0)
        print x[0]
    print 'after 100 coast', x
    x, P = ekf.update_centerline(x, P, 0, 0.01, 0.1, np.eye(3) * 0.1)
    print x


if __name__ == '__main__':
    ekf_test()
