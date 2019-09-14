import chunk
import numpy as np
import struct


def read_header(f):
    try:
        ck = chunk.Chunk(f, False, False, True)
    except EOFError:
        return False, None
    if ck.getname() == b'cfg1':
        hdr = ck.read()
        print("read header: ", struct.unpack("=%dh" % (len(hdr)/2), hdr))
        return True, hdr
    if ck.getname() == b'CYCF':
        f.seek(0)
        return True, None
    else:
        print("Not a cycloid IFF log file (got ", ck.getname(), "?)")
        return False, None


def read_frame(f):
    try:
        ck = chunk.Chunk(f, False, False, True)
    except EOFError:
        return False, None
    if ck.getname() != b'CYCF':
        print("Not a cycloid IFF log file (got ", ck.getname(), "?)")
        return False, None

    # read timestamp header
    ts = struct.unpack("=II", ck.read(8))

    framedata = {
        'tstamp': ts[0] + ts[1] / 1000000.
    }

    # read all embedded chunks
    while True:
        try:
            ick = chunk.Chunk(ck, False, False, True)
        except EOFError:
            break
        n = ick.getname()
        if n == b'CSta':  # car state
            data = struct.unpack("=bbffffffBHHHHHHHH", ick.read())
            throttle, steering = data[0:2]
            accel = np.float32(data[2:5])
            gyro = np.float32(data[5:8])
            servo = data[8]
            wheels = np.uint16(data[9:13])
            periods = np.uint16(data[13:17])
            framedata['carstate'] = (
                throttle, steering, accel, gyro, servo, wheels, periods)
        if n == b'CSt1':  # car state
            data = struct.unpack("=bbffffffff", ick.read())
            throttle, steering = data[0:2]
            accel = np.float32(data[2:5])
            gyro = np.float32(data[5:8])
            wheeldist = np.float32(data[8:12])
            wheelv = np.float32(data[12:16])
            framedata['carstate'] = (
                throttle, steering, accel, gyro, 0, wheeldist, wheelv)
        # monte carlo localization, 4-float state (particles w/ heading)
        elif n == b'MCL4':
            framedata['particles'] = np.frombuffer(
                ick.read(), np.float32).reshape((-1, 4))
        elif n == b'aCDF':  # activation CDF, new thing
            framedata['activations'] = np.frombuffer(ick.read(), np.int32)
        elif n == b'LM01':  # expected landmark location
            numL, = struct.unpack('B', ick.read(1))
            c0c1 = np.frombuffer(ick.read(), np.uint16).reshape((-1, numL))
            nP = c0c1.shape[0] // 2
            framedata['c0'] = c0c1[:nP]
            framedata['c1'] = c0c1[nP:]
        elif n == b'CTLs':  # controller state
            framedata['controldata'] = struct.unpack("=17f", ick.read())
        elif n == b'CTL2':  # controller state
            dat = ick.read()
            framedata['controldata2'] = struct.unpack("=%df" % (len(dat)/4), dat)
        elif n == b'Y420':  # YUV420 frame
            w, = struct.unpack('=H', ick.read(2))
            framedata['yuv420'] = np.frombuffer(
                ick.read(), np.uint8).reshape((-1, w))
        else:
            ick.skip()

    return True, framedata


class RecordIterator:
    def __init__(self, f):
        self.f = f
        _, self.header = read_header(f)

    def __iter__(self):
        return self

    def __next__(self):
        ok, data = read_frame(self.f)
        if not ok:
            raise StopIteration
        return data

    def next(self):
        return self.__next__()


class RecordScanner:
    def ScanIndex(self):
        idx = []
        while True:
            idx.append(self.f.tell())
            try:
                ck = chunk.Chunk(self.f, False, False, True)
            except EOFError:
                return idx[:-1]
            if ck.getname() != b'CYCF':
                raise Exception("Not a cycloid IFF log file (got " +
                                ck.getname() + "?)")
            ck.skip()

    def __init__(self, f):
        self.f = f
        _, self.header = read_header(f)
        self.idx = self.ScanIndex()

    def num_frames(self):
        return len(self.idx)

    def frame(self, i):
        self.f.seek(self.idx[i], 0)
        ok, data = read_frame(self.f)
        if not ok:
            raise Exception("failed reading frame %d @ offset %d?" % (
                i, self.idx[i]))
        return data
