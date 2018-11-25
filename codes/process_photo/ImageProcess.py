import cv2
import numpy as np
from matplotlib import pyplot as plt


class ImageProcess:

	def __init__(self, baseDir, imgName):
		#print(baseDir)
		self.baseDir = baseDir
		self.imgName = imgName
		self.extension = '.jpg'
		self.canny1 = "-canny-processed-1"
		self.canny2 = "-canny-processed-2"
		self.lap1 = "-laplacianed-1"

		self.cannyProcess1()
		self.cannyProcess2()
		self.laplacianProcess()

	def cannyProcess1(self):
		index = self.imgName.index('.')
		editName = self.imgName[:index] + self.canny1
		saveDir = self.baseDir + editName + self.extension

		img = cv2.imread(self.baseDir + self.imgName)
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		edges = cv2.Canny(gray,50,120)
		cv2.imwrite(saveDir, edges)

	def cannyProcess2(self):
		index = self.imgName.index('.')
		editName = self.imgName[:index] + self.canny2
		saveDir = self.baseDir + editName + self.extension

		img = cv2.imread(self.baseDir + self.imgName)
		lower_red = np.array([40, 100, 50])
		upper_red = np.array([90, 255, 255])

		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, lower_red, upper_red)
		res = cv2.bitwise_and(img, img, mask=mask)

		edges = cv2.Canny(res, 100, 200)
		cv2.imwrite(saveDir, edges)

	def laplacianProcess(self):
                index = self.imgName.index('.')
                editName = self.imgName[:index] + self.lap1
                saveDir = self.baseDir + editName + self.extension

		img0 = cv2.imread(self.baseDir + self.imgName,)
#		img0 = cv2.imread('../../resources/kkm-base-photos/red-shadow.jpg',)

# converting to gray scale
		gray = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)

# remove noise
		img = cv2.GaussianBlur(gray,(3,3),0)

# convolute with proper kernels
		laplacian = cv2.Laplacian(img,cv2.CV_64F)

		plt.imsave(saveDir, laplacian, cmap='gray')

