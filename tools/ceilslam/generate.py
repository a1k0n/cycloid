# -*- coding: utf-8 -*-
from __future__ import print_function
import ceiltrack
import numpy as np
import struct


def rle(mask):
    # assume run starts with zeros, output <skip length> <run length>
    out = []

    zero = True
    run = 0
    for i in range(len(mask)):
        if zero:
            if mask[i]:
                out.append(run)
                run = 1
                zero = False
            else:
                run += 1
        else:
            if mask[i]:
                run += 1
            else:
                out.append(run)
                run = 1
                zero = True
    return np.array(out)


def main():
    ceilmask, pts = ceiltrack.ceillut()

    fname = "ceillut.bin"
    f = open(fname, "wb")
    # header:
    #  - uint16 image height
    #  - uint16 image width
    #  - uint32 number of pixels in mask (N)
    #  - uint32 rle-compressed mask size in bytes (M)
    # followed by
    # [uint16 skip, uint16 run] x (M/4)
    # [fp16 u, fp16 v] x N
    rlemask = rle(ceilmask.reshape(-1)).astype(np.uint16)
    h, w = ceilmask.shape
    hlen = 2 + 2 + 4 + 4
    f.write(struct.pack("=4sIHHII", b'cmLU', hlen, h, w,
                        np.sum(ceilmask), len(rlemask)))
    f.write(rlemask.tobytes())
    f.write(pts.T.astype(np.float16).tobytes())
    f.close()
    print("wrote", fname, 'mask', len(rlemask)*2, 'bytes; pts x', pts.T.shape, '=',
          pts.shape[1]*4, 'bytes', np.sum(ceilmask))


if __name__ == '__main__':
    main()
