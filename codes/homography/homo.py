import cv2
import numpy as np

#img1_top = np.float32([[400,724], [606,691], [620,1035],[228,349]])
#img2_side = np.float32([[412,561], [625,530], [708,793], [282,373]])

img1_top = np.float32([[229,349], [214,1060], [732,1091],[753,366]])
img2_side = np.float32([[283,373], [126,863], [885,823], [706,358]])

h, mask = cv2.findHomography(img2_side, img1_top) #(src, dst)
im = cv2.imread("side.jpg")
out = cv2.warpPerspective(im, h, (960,1280))
cv2.imwrite("result.png", out)
