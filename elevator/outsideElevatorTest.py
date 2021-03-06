#!/usr/bin/env python
import cv2
import cv2.cv as cv
import numpy as np
from matplotlib import pyplot as plt
import string
import sys

# I guess I'm prototyping here and if it works, I'll translate it into c++???

getNewHists = False

# loads stringified numpy array histogram from file 
def loadHistFromFile(filename):

	g = open(filename, 'r')

	hist = np.zeros(256)

	# quickly concatenate the file
	str_list = []
	for line in g:
		str_list.append(line)

	histoString = string.join(str_list, '')
	histoString = histoString[2:len(histoString)-1].split()

	for i in range(len(histoString)):
		
		hist[i] = float(histoString[i])


	hist = np.reshape(hist, (256, 1))

	# plt.plot(hist)
	# plt.xlim([0,256])
	# plt.show()	

	g.close()
	return hist.astype(float)


# METHOD #3: ROLL YOUR OWN
def chi2_distance(histA, histB, eps = 1e-10):
	# compute the chi-squared distance
	d = 0.5 * np.sum([((a - b) ** 2) / (a + b + eps)
		for (a, b) in zip(histA, histB)])
 
	# return the chi-squared distance
	return d


def main():

	if len(sys.argv[1:]) == 0:
		getNewHists = False
	else:
		getNewHists = True

	img = cv2.imread('pics/outside2.jpg',0)

	# For static image
	rows,cols = img.shape
	M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
	img = cv2.warpAffine(img,M,(cols,rows))

	img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
	img = cv2.medianBlur(img,5)
	cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

	height = img.shape[0]
	width = img.shape[1]

	circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=70, param2=60) 
	print circles

	filename = 'modelButtonHistogram.txt'
	modelButtonHist = loadHistFromFile(filename)


	if getNewHists:
		f = open("sampleHistograms.txt", "w")

	
	circles = np.uint16(np.around(circles))

	totalRadius = 1.0 * np.sum(circles[0], axis=0)
	meanRadius = int(totalRadius[2] / len(circles[0]))

	print "meanRadius = ", meanRadius

	compHistResults = []

	# draw all circles
	for i in circles[0,:]:

		circRadius = i[2]
		cx = i[0]
		cy = i[1]
		
		pt1 = (cx - meanRadius, cy - 10)
		pt2 = (cx + meanRadius, cy + 10)
		mask = np.zeros((height, width), np.uint8)
		cv2.rectangle(mask, pt1, pt2, (255, 255, 255), thickness=cv.CV_FILLED)

		hist1 = cv2.calcHist([cimg], [0], mask, [256], [0,256])
		# plt.plot(hist1)
		# plt.xlim([0,256])
		# plt.show()	

		print type(hist1[0][0])

		# compare histogram with model button histogram 
		# ret = cv2.compareHist(hist1, modelButtonHist, cv.CV_COMP_CHISQR)

		ret = chi2_distance(hist1, modelButtonHist)
		compHistResults.append(ret)

		print "result: ", ret

		if getNewHists:
			hist1 = hist1.astype(int)
			hist1 = hist1.flatten()
			f.write(str(hist1) + "\n\n")
		

		# draw the outer circle
		cv2.circle(cimg,(cx, cy),circRadius,(0,255,0),2)
		# draw the center of the circle
		cv2.circle(cimg,(cx, cy),2,(0,0,255),3)
		print i


	if getNewHists:
		f.close()


	# TODO: Add logic here to find the correct button!




	# # print candidates

	# # draw out the elevator floor buttons
	# for circ in candidates:
	# 	cv2.circle(cimg, (circ[0], circ[1]), circ[2], (255, 0, 0), 3)

	# cv2.imshow('detected circles',cimg)
	cv2.imshow('detected circles',cimg)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
