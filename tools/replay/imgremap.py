import numpy as np

ytop = 100  # discard image above y=ytop (320x240 images)


def load():
    udplane = np.load("udplane.npy")
    udmask = np.load("udmask.npy")
    bucketcount = np.load("bucketcount.npy")
    floodmap = np.load("floodmap.npy")
    return udplane, udmask, bucketcount, floodmap
