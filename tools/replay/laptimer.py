# 450x160
import recordreader

FINISHX = 7.4
FINISHY = 150/50.0


def laptimer(fname, f):
    px, py = None, None
    prevlap = None
    lapcount = 0
    for rec in recordreader.RecordIterator(f):
        ts = rec['tstamp']
        x, y = rec['controldata2'][:2]
        if px is not None:
            if px < FINISHX and x >= FINISHX and y < FINISHY:
                if prevlap is not None:
                    lapcount += 1
                    print("%s lap %d: %0.2f" % (fname, lapcount, ts-prevlap))
                prevlap = ts
        px, py = x, y


if __name__ == '__main__':
    import sys
    for fname in sys.argv[1:]:
        with open(fname, "rb") as f:
            laptimer(fname, f)
