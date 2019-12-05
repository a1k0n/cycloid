import cv2

m = cv2.imread("map.png")
mres = cv2.resize(m, (320, 160), interpolation=cv2.INTER_AREA)
res420 = cv2.cvtColor(mres, cv2.COLOR_BGR2YUV_I420)
open("map.yuv420", "wb").write(res420.tobytes())
