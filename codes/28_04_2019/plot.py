#!/usr/bin/env python
from matplotlib import pyplot
from pylab import genfromtxt

mat1 = genfromtxt("/home/pi/Desktop/with_c++_AllinOneBoostx2/testFilter.txt")
pyplot.plot(mat1[:,0], label="original")
pyplot.plot(mat1[:,1], label="kalmanPredicted")
pyplot.legend()
pyplot.show()

#mat1 = genfromtxt("/home/pi/Desktop/with_c++/testFilter.txt")
#pyplot.plot(mat1[:,0], label="original")
#pyplot.plot(mat1[:,1] - mat1[:,0], label="kalmanPredicted")
#pyplot.legend()
#pyplot.show()

#mat2 = genfromtxt("/home/pi/Desktop/with_c++/test.txt")
#pyplot.plot(mat1[:,2], label="original")
#pyplot.plot(mat1[:,3], label="kalmanPredicted")
#pyplot.legend()
#pyplot.show()
