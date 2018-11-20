import cv2
import numpy as np



img = cv2.imread('../../resources/kkm-base-photos/red-shadow.jpg', cv2.IMREAD_GRAYSCALE)
rows,cols = img.shape
denoised = cv2.GaussianBlur(img,(5,5),0)
filtered = cv2.Laplacian(denoised,cv2.CV_64F)
cv2.imwrite('../../resources/kkm-base-photos/re1.jpg',filtered)

img = cv2.imread('../../resources/kkm-base-photos/darkblue-light.jpg',cv2.IMREAD_GRAYSCALE)
rows,cols = img.shape
 
denoised = cv2.GaussianBlur(img,(5,5),0)
filter = cv2.Laplacian(denoised,cv2.CV_64F)

cv2.imwrite('../../resources/kkm-base-photos/re2.jpg',filtered)

img0 = cv2.imread('../../resources/kkm-base-photos/red-shadow.jpg',)

# converting to gray scale
gray = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)

# remove noise
img = cv2.GaussianBlur(gray,(3,3),0)

# convolute with proper kernels
laplacian = cv2.Laplacian(img,cv2.CV_64F)

