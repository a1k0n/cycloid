import numpy as np
import imgremap


udplane, udmask, bucketcount, floodmap = imgremap.load()
uxrange = np.min(udplane[udmask, 0]), np.max(udplane[udmask, 0]) + 1
uyrange = np.min(udplane[udmask, 1]), np.max(udplane[udmask, 1]) + 1
pixel_scale_m = 0.025


def reproject(img):
    x0, y0 = uxrange[0], uyrange[0]
    # uint16 maybe? could overflow on certain pixels
    m = np.zeros((uyrange[1] - y0, uxrange[1] - x0), np.float32)
    np.add.at(m, (udplane[udmask, 1] - y0, udplane[udmask, 0] - x0),
              img[-240 + imgremap.ytop:][udmask])
    m *= bucketcount
    m[bucketcount == 0] = m[(floodmap[:, 0], floodmap[:, 1])]
    return m
    
def tophat(m):
    ''' So-called "top hat filter" which is like two nested boxcar
    filters: it's a convolution with [-1, -1, 2, 2, -1, -1].

    The rationale is that we have a 50mm-wide line we want to detect,
    so we first remap the image into a birdseye view with each pixel being
    placed into a 25mmx25mm bucket. Using exactly two "pixels" per line allows
    us to find the relative "phase" between pixels, kind of like in quadrature
    encoders.

    So first we add each incoming image pixel into its corresponding bucket,
    and then average it by the bucket count (which is always fixed)

    Then we fill in all the missing values with their nearest neighbors via
    floodmap, so that when we convolve we don't have false edges detected.

    Then we run the convolution, and use a heuristic linear classifier on the
    activations for the YUV channels.  Anything > 0 is kept and the activation
    strength is used as weights in the linear regression to find the centerline
    curve.

    This may need adjustment to reject white lines if we're only looking for
    yellow lines.

    input is a YUV 320x240 array
    '''

    x0, y0 = uxrange[0], uyrange[0]

    # convolve [-1, -1, 2, 2, -1, -1] with image
    hv = np.cumsum(m, axis=1)
    hv = -(hv[:, 6:] - hv[:, :-6]) + 3*(hv[:, 4:-2] - hv[:, 2:-4])
    hv[bucketcount[:, 3:-3] == 0] = 0

    # detected = (0.25*hv[:, :, 0] - 2*hv[:, :, 1] + 0.5*hv[:, :, 2] - 30)

    detected = -hv - 18
    # detected = -hv - 20
    

    # hh = np.cumsum(m, axis=0)
    # hh = -(hh[6:, :] - hh[:-6, :]) + 3*(hh[4:-2, :] - hh[2:-4, :])
    # , (hh * 0.2 + 128).astype(np.uint8)[:, :]
    return hv, (np.clip(detected, 0, 255)).astype(np.uint8)[:, :]


def detect_centerline(img):
    hv, th = tophat(img)

    # quadratic regression
    indices = np.nonzero(th)
    N = len(indices[0])
    if N > 10:
        s = pixel_scale_m
        w = th[indices]  # * bucketcount[:, 3:-3][indices]
        X = np.vstack([w*s*s*indices[0]**2, w*s*indices[0], w]).T
        y = w*s*(indices[1] - th.shape[1] / 2)
        yc = np.sum(w * s * indices[0]) / np.sum(w)
        XTX = np.dot(X.T, X)
        XTy = np.dot(X.T, y)
        XTXinv = np.linalg.inv(XTX)
        # optimum
        B = np.dot(XTXinv, XTy)
        # squared residuals
        r2 = np.dot(B.T, np.dot(XTX, B)) - 2*np.dot(B, XTy) + np.dot(y.T, y)
        # covariance matrix
        Rk = np.zeros((4, 4))
        Rk[:3, :3] = XTXinv * r2
        # covariance of y_c
        Rk[3, 3] = XTX[1, 1] / np.sum(w) - yc**2
        Rk /= N
        return hv, th, B, yc, Rk

    return hv, th, None, None, None
