from PIL import Image
import numpy as np

DPI = 600  # printer DPI
WHEEL_ID = 67.9  # wheel inner diameter (mm)
WHEEL_WIDTH = 26
TICKS_PER_REV = 10

wheel_id_px = int(np.round(WHEEL_ID * np.pi * DPI / 25.4))
wheel_w_px = int(np.round(WHEEL_WIDTH * DPI / 25.4))

print wheel_w_px, wheel_id_px

img = np.zeros((wheel_id_px, wheel_w_px*4 + 3), np.uint8)

for i in range(TICKS_PER_REV):
    # make the top half of each period white
    y0 = int(i * float(wheel_id_px) / TICKS_PER_REV)
    y1 = int((i+0.5) * float(wheel_id_px) / TICKS_PER_REV)
    img[y0:y1] = 255


img[:, wheel_w_px:wheel_w_px+4] = 255
img[:, wheel_w_px*2:wheel_w_px*2+4] = 255
img[:, wheel_w_px*3:wheel_w_px*3+4] = 255

Image.fromarray(img).save("we.png")
