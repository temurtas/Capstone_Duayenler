import cv2
import numpy as np
import matplotlib.pyplot as plt

IMAGE_H = 180
IMAGE_W = 640

src = np.float32([[0, IMAGE_H], [200, IMAGE_H], [0, 0], [IMAGE_W, 0]])
dst = np.float32([[100, IMAGE_H], [100, IMAGE_H], [100, 0], [100+IMAGE_W, 0]])
M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

img = cv2.imread('12.jpg') # Read the test img
#img = img[80:(80+IMAGE_H), 100:IMAGE_W] # Apply np slicing for ROI crop
warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
plt.show()
