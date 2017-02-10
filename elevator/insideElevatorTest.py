from collections import Counter
import cv2
import cv2.cv as cv
import math
from matplotlib import pyplot as plt
import numpy as np
import sys
import string
import time

def positionBasedLogic():

	# img = cv2.imread('pics/gretchenTest.jpg')
	img = cv2.imread('pics/inside1.jpg')


	img = cv2.medianBlur(img,5)
	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	# For static image
	rows,cols = img.shape
	M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
	img = cv2.warpAffine(img,M,(cols,rows))

	print "original shape: ", img.shape[:2]

	idealx = 888
	idealy = 888

	width = img.shape[0]
	height = img.shape[1]

	print "w, h: ", width, height

	xratio = (1.0 * idealx)/width
	yratio = (1.0 * idealy)/height

	print "ratios: ", xratio, yratio

	# img = cv2.resize(img, (0, 0), fx=0.35, fy=0.35)
	img = cv2.resize(img, (0, 0), fx=xratio, fy=yratio)

	print "after adjustment shape: ", img.shape[:2]


	# cv2.imshow('img',img)
	# cv2.waitKey(0)


	#good for 0.35 resize of phone camera picture
	# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=110, param2=95)

	# good for 0.3 resize of phone camera picture
	circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=380) 
	# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=100) 

	# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=80, param2=75) 
	print "number of circles is: ", circles.shape[1]

	print "circles is: ", circles, "shape: ", circles.shape[:2], "shape: ", circles.shape

	if circles.shape[1]:
		circles = np.uint16(np.around(circles))
		toSortCircles = np.int16(np.around(circles))

		for i in circles[0,:]:
		    # draw the outer circle
		    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
		    # draw the center of the circle
		    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

		# debug
		cv2.imshow('detected circles',cimg)
		cv2.waitKey(0)


		# want to sort by y so that we can figure out the layers
		sortedCircle = toSortCircles[0]
		# print "before sorting", sortedCircle

		sortedCircle = sortedCircle[sortedCircle[:,1].argsort()]
		# print "after sorting", sortedCircle

		candidates = []
		numCircles = sortedCircle.shape[0]


		for i in range(1, numCircles):

			# if y for this circle is far away from y of last circle,
			# that means that this is a new row of circles and this might 
			# be a candidate
			if abs(sortedCircle[i][1] - sortedCircle[i-1][1]) > 0.1 * sortedCircle[i-1][1]:
				# check if this circle has a neighbour that is on the same row.
				# if no neighbour on the same row, then add to candidates
				
				# print "\n large delta y, checking neighbours: \n"
				if i != numCircles - 1: # this is not the last circle:

					if abs(sortedCircle[i][1] - sortedCircle[i+1][1]) > 0.1 * sortedCircle[i-1][1]:
						candidates.append(sortedCircle[i])

			# if this is the last circle: 
			if i == numCircles - 1:
				
				#check to see if this circle is a candidate
				if len(candidates) != 0:
					if abs(sortedCircle[i][0] - candidates[0][0]) < 0.1 * candidates[0][0]:
						candidates.append(sortedCircle[i])
			
		print "candidates: ", candidates


		print "len is now:", len(candidates)

		badIndices = []
		badOnes = []

		# print "\n\n here \n\n"

		# run through candidates to make sure they have similar x 
		for i in range(1, len(candidates)):
			# check that the buttons are vertically aligned
			if abs(candidates[i][0] - candidates[i-1][0]) > 0.05 * candidates[i-1][0]:
				badIndices.append(i)
				badOnes.append(candidates[i])

		print badIndices
		print badOnes

		if badIndices != []:
			# for bad in badOnes:
			# 	candidates.remove(bad.any())

			for bad in badIndices:
				print "in bad:", bad
				candidates.pop(bad)

		print "printing candidates"
		print candidates

		numCandidates = len(candidates)
		i = numCandidates

		for circ in candidates:
			cv2.circle(cimg, (circ[0], circ[1]), circ[2], (255, 0, 0), 3)
			print (circ[0] - 0.25*circ[2], circ[1] - 0.25*circ[2])
			font = cv2.FONT_HERSHEY_SIMPLEX
			# cv2.putText(cimg, "lala", (2.0, 2.0), font, 3, (255, 0, 0))


			cv2.putText(cimg, str(i), (int(circ[0] - 0.25*circ[2]), int(circ[1] + 0.25*circ[2])), font, 1, (255, 0, 0), thickness = 2)
			i -= 1


		cv2.imshow('detected circles',cimg)
		cv2.waitKey(0)
	else:
		print "no circles detected"
	cv2.destroyAllWindows()





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


def histogramMethod():
	img = cv2.imread('pics/inside1.jpg')

	img = cv2.medianBlur(img,5)
	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	# For static image
	rows,cols = img.shape
	M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
	img = cv2.warpAffine(img,M,(cols,rows))

	# print img.shape[:2]

	# img = cv2.resize(img, (0, 0), fx=0.35, fy=0.35)
	img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)


	height = img.shape[0]
	width = img.shape[1]

	#good for 0.35 resize of phone camera picture
	# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=110, param2=95)

	# good for 0.3 resize of phone camera picture
	# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=85) 

	circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=75, param2=71) 

	circles = np.uint16(np.around(circles))
	

	# debug
	# cv2.imshow('detected circles',cimg)
	# cv2.waitKey(0)

	# start timer

	filename = 'modelButtonHistogram.txt'
	modelButtonHist = loadHistFromFile(filename)


	candidates = []


		
	circles = np.uint16(np.around(circles))

	totalRadius = 1.0 * np.sum(circles[0], axis=0)
	meanRadius = int(totalRadius[2] / len(circles[0]))

	print "meanRadius = ", meanRadius

	compHistResults = []

	# draw all circles
	for i in circles[0,:]:
			    # draw the outer circle

		circRadius = i[2]
		cx = i[0]
		cy = i[1]
		
		cv2.circle(cimg,(cx,cy),circRadius,(0,255,0),2)
		# draw the center of the circle
		cv2.circle(cimg,(cx,cy),2,(0,0,255),3)

		cv2.imshow('detected circles',cimg)
		cv2.waitKey(0)

		pt1 = (cx - meanRadius, cy - 10)
		pt2 = (cx + meanRadius, cy + 10)
		mask = np.zeros((height, width), np.uint8)
		cv2.rectangle(mask, pt1, pt2, (255, 255, 255), thickness=cv.CV_FILLED)

		hist1 = cv2.calcHist([cimg], [0], mask, [256], [0,256])
		plt.plot(hist1)
		plt.xlim([0,256])

		plt.show()	

		print type(hist1[0][0])

		# compare histogram with model button histogram 
		# ret = cv2.compareHist(hist1, modelButtonHist, cv.CV_COMP_CHISQR)

		ret = chi2_distance(hist1, modelButtonHist)
		compHistResults.append(ret)

		print "result: ", ret

		# draw the outer circle
		cv2.circle(cimg,(cx, cy),circRadius,(0,255,0),2)
		# draw the center of the circle
		cv2.circle(cimg,(cx, cy),2,(0,0,255),3)
		print i


	print compHistResults
	

	asdfasdf
	# not finished yet because histogram method doesn't work very well with
	# buttons inside elevator. 

	# stop timer



	print "printing candidates"
	print candidates

	i = 3

	for circ in candidates:
		cv2.circle(cimg, (circ[0], circ[1]), circ[2], (255, 0, 0), 3)
		print (circ[0] - 0.25*circ[2], circ[1] - 0.25*circ[2])
		font = cv2.FONT_HERSHEY_SIMPLEX
		# cv2.putText(cimg, "lala", (2.0, 2.0), font, 3, (255, 0, 0))


		cv2.putText(cimg, str(i), (int(circ[0] - 0.25*circ[2]), int(circ[1] + 0.25*circ[2])), font, 1, (255, 0, 0), thickness = 2)
		i -= 1


	cv2.imshow('detected circles',cimg)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


class buttonTemplateMatching:

	def __init__(self):
		# load templates of elevator buttons
		self.origButtonTemplates = {}
		self.buttonTemplates = {}
		button1 = []
		button2 = []
		button3 = []

		templateFile = open('templatePics/testButtonTemplates.txt', 'r')
		pathPrefix = 'templatePics/'
		
		radiusTotal = 0
		numRadius = 0

		for picname in templateFile:
			picPath = (pathPrefix + picname).strip()
			if picPath == pathPrefix:
				continue
			pic = cv2.imread(picPath)
			
			if pic is None:
				print picname, "did not load properly"
				continue

			pic = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)

			tempx, tempy = pic.shape[:2]
			print tempx, tempy
			maxdim = max(tempx, tempy)

			
			if "1" in picname:
				button1.append(pic)
			elif "2" in picname:
				button2.append(pic)
			elif "3" in picname:
				button3.append(pic)

			print "Loaded: ", picname

			circles = cv2.HoughCircles(pic, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=135, param2=130, maxRadius=maxdim/2) 
			print circles

			if circles is not None:
				circles = np.uint16(np.around(circles))
				toSortCircles = np.int16(np.around(circles))

				if len(circles[0]) == 1:
					radiusTotal += circles[0][0][2]
					numRadius += 1
				else:
					sumRad = 0
					for i in circles[0,:]:
						sumRad += i[2]
					avg = sumRad / len(circles[0,:])
					radiusTotal += avg
					numRadius += 1

				#debug
				# for i in circles[0,:]:
				# 	# draw the outer circle
				# 	cv2.circle(pic,(i[0],i[1]),i[2],(0,255,0),2)
				# 	# draw the center of the circle
				# 	cv2.circle(pic,(i[0],i[1]),2,(0,0,255),3)

				# debug
				# cv2.imshow('detected circles',pic)
				# cv2.waitKey(0)
		
		self.avgTemplateRadius = radiusTotal / numRadius
		self.currAvgTemplateRadius = radiusTotal / numRadius
		print "avgTemplateRadius", self.avgTemplateRadius

		self.origButtonTemplates[1] = button1
		self.origButtonTemplates[2] = button2
		self.origButtonTemplates[3] = button3
		self.buttonTemplates[1] = button1
		self.buttonTemplates[2] = button2
		self.buttonTemplates[3] = button3



	def resizeTemplatePics(self, avgDetectedRadius):
		print "currAvgTemplateRadius", self.currAvgTemplateRadius

		ratio = float(avgDetectedRadius)/self.avgTemplateRadius

		for i in range(1, 4):
			button = self.origButtonTemplates[i]
			resizedButton = []
			for templatePic in button:
				# debug
				# cv2.imshow('templatePic',templatePic)
				# cv2.waitKey(0)
				# print "before size: ", templatePic.shape[:2]

				templatePic = cv2.resize(templatePic, (0, 0), fx=ratio, fy=ratio)

				# debug
				# cv2.imshow('templatePic',templatePic)
				# cv2.waitKey(0)
				# print "after size: ", templatePic.shape[:2]

				resizedButton.append(templatePic)

			self.buttonTemplates[i] = resizedButton

		self.currAvgTemplateRadius = avgDetectedRadius


	def matchButtonImage(self):
		# first id circles

		# For static image
		img = cv2.imread('pics/inside1.jpg')

		img = cv2.medianBlur(img,5)
		img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		rows,cols = img.shape
		M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
		img = cv2.warpAffine(img,M,(cols,rows))

		print "original shape: ", img.shape[:2]

		idealx = 888
		idealy = 888

		width = img.shape[0]
		height = img.shape[1]

		print "w, h: ", width, height

		xratio = (1.0 * idealx)/width
		yratio = (1.0 * idealy)/height

		print "ratios: ", xratio, yratio

		# img = cv2.resize(img, (0, 0), fx=0.35, fy=0.35)
		img = cv2.resize(img, (0, 0), fx=xratio, fy=yratio)

		print "after adjustment shape: ", img.shape[:2]

		#good for 0.35 resize of phone camera picture
		# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=110, param2=95)

		# good for 0.3 resize of phone camera picture
		circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=180) 
		# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=100) 

		# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=80, param2=75) 
		print "number of circles is: ", circles.shape[1]

		print "circles is: ", circles, "shape: ", circles.shape[:2], "shape: ", circles.shape

		colorimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

		radiusList = []
		newMeanCircleRadius = 0

		if circles is not None:
			circles = np.uint16(np.around(circles))
			toSortCircles = np.int16(np.around(circles))

			for i in circles[0,:]:
				# draw the outer circle
				cv2.circle(colorimg,(i[0],i[1]),i[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(colorimg,(i[0],i[1]),2,(0,0,255),3)

				print "circle radius: ", i[2]
				radiusList.append(i[2])

			# debug
			# cv2.imshow('detected circlesla',colorimg)
			# cv2.waitKey(0)


			meanCircleRadius = np.mean(radiusList)
			stdCircleRadius = np.std(radiusList)
			print meanCircleRadius
			print stdCircleRadius
			
			radiusListOutliersRemoved = []
			for rad in radiusList:
				if rad < meanCircleRadius + stdCircleRadius and \
					rad > meanCircleRadius - stdCircleRadius:

					radiusListOutliersRemoved.append(rad)

			print radiusListOutliersRemoved
			newMeanCircleRadius = np.mean(radiusListOutliersRemoved)
			print "new mean:", newMeanCircleRadius


		#resize template pic so that button is similar size
		if 0.8*newMeanCircleRadius > self.currAvgTemplateRadius or \
			0.8*self.currAvgTemplateRadius > newMeanCircleRadius:
			self.resizeTemplatePics(newMeanCircleRadius)

		# now implement feature detection

		thresh = 0.92
		r = int(newMeanCircleRadius)
		
		buttonNumber = 0

		for i in range(1, 4):
			button = self.buttonTemplates[i]
			buttonNumber += 1
			for templatePic in button:

				tempx, tempy = templatePic.shape[:2]
				sz = max(tempx, tempy)
				res = cv2.matchTemplate(img, templatePic, cv2.TM_CCORR_NORMED)

				resdisplay = (res-res.min())/(res.max()-res.min())
				

				while 1:
					minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(res)
					print "stats:", minVal, maxVal, minLoc, maxLoc

					resdisplay1 = (res-res.min())/(res.max()-res.min())
					cv2.imshow("resdisplay1", resdisplay1)
					cv2.waitKey()

					if maxVal < thresh:

						break

					else:

						x,y = maxLoc

						p0 = (x-r, y-r)
						p1 = (x+r, y+r)
						cv2.rectangle(res, p0, p1, (0,0,0), -1) # zero out on res

						pa = (x,y)
						pb = (x+sz, y+sz)
						cv2.rectangle(colorimg, pa, pb, (0,0,255), 1)

				cv2.imshow("resdisplay", resdisplay)
				cv2.waitKey()

				cv2.imshow("colorimg", colorimg)
				cv2.waitKey()


class numberTemplateMatching:

	def __init__(self):
		# load templates of elevator buttons
		self.origButtonTemplates = {}
		self.buttonTemplates = {}
		button1 = []
		button2 = []
		button3 = []

		templateFile = open('templatePics/testNumberTemplates.txt', 'r')
		pathPrefix = 'templatePics/'
		
		heightTotal = 0
		numHeights = 0

		for picname in templateFile:
			picPath = (pathPrefix + picname).strip()
			if picPath == pathPrefix:
				continue
			pic = cv2.imread(picPath)

			if pic is None:
				print picname, "did not load properly"
				continue

			pic = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)

			tempx, tempy = pic.shape[:2]
			print tempy
			heightTotal += tempy
			numHeights += 1

			
			if "1" in picname:
				button1.append(pic)
			elif "2" in picname:
				button2.append(pic)
			elif "3" in picname:
				button3.append(pic)

			print "Loaded: ", picname
		
		self.avgTemplateRadius = heightTotal / numHeights
		self.currAvgTemplateRadius = self.avgTemplateRadius
		print "avgTemplateRadius", self.avgTemplateRadius

		self.origButtonTemplates[1] = button1
		self.origButtonTemplates[2] = button2
		self.origButtonTemplates[3] = button3
		self.buttonTemplates[1] = button1
		self.buttonTemplates[2] = button2
		self.buttonTemplates[3] = button3



	def resizeTemplatePics(self, avgDetectedRadius):
		print "currAvgTemplateRadius", self.currAvgTemplateRadius

		ratio = float(avgDetectedRadius)/self.avgTemplateRadius

		for i in range(1, 4):
			button = self.origButtonTemplates[i]
			resizedButton = []
			for templatePic in button:
				# debug
				# cv2.imshow('templatePic',templatePic)
				# cv2.waitKey(0)
				# print "before size: ", templatePic.shape[:2]

				templatePic = cv2.resize(templatePic, (0, 0), fx=ratio, fy=ratio)

				# debug
				# cv2.imshow('templatePic',templatePic)
				# cv2.waitKey(0)
				# print "after size: ", templatePic.shape[:2]

				resizedButton.append(templatePic)

			self.buttonTemplates[i] = resizedButton

		self.currAvgTemplateRadius = avgDetectedRadius


	def matchNumberImage(self):
		# first id circles

		# For static image
		img = cv2.imread('pics/inside1.jpg')

		img = cv2.medianBlur(img,5)
		img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		rows,cols = img.shape
		M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
		img = cv2.warpAffine(img,M,(cols,rows))

		print "original shape: ", img.shape[:2]

		idealx = 888
		idealy = 888

		width = img.shape[0]
		height = img.shape[1]

		print "w, h: ", width, height

		xratio = (1.0 * idealx)/width
		yratio = (1.0 * idealy)/height

		print "ratios: ", xratio, yratio

		# img = cv2.resize(img, (0, 0), fx=0.35, fy=0.35)
		img = cv2.resize(img, (0, 0), fx=xratio, fy=yratio)

		print "after adjustment shape: ", img.shape[:2]

		#good for 0.35 resize of phone camera picture
		# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=110, param2=95)

		# good for 0.3 resize of phone camera picture
		circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=180) 
		# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=100) 

		# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=80, param2=75) 
		print "number of circles is: ", circles.shape[1]

		print "circles is: ", circles, "shape: ", circles.shape[:2], "shape: ", circles.shape

		colorimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

		radiusList = []
		newMeanCircleRadius = 0

		if circles is not None:
			circles = np.uint16(np.around(circles))
			toSortCircles = np.int16(np.around(circles))

			for i in circles[0,:]:
				# draw the outer circle
				cv2.circle(colorimg,(i[0],i[1]),i[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(colorimg,(i[0],i[1]),2,(0,0,255),3)

				print "circle radius: ", i[2]
				radiusList.append(i[2])

			# debug
			# cv2.imshow('detected circlesla',colorimg)
			# cv2.waitKey(0)


			meanCircleRadius = np.mean(radiusList)
			stdCircleRadius = np.std(radiusList)
			print meanCircleRadius
			print stdCircleRadius
			
			radiusListOutliersRemoved = []
			for rad in radiusList:
				if rad < meanCircleRadius + stdCircleRadius and \
					rad > meanCircleRadius - stdCircleRadius:

					radiusListOutliersRemoved.append(rad)

			print radiusListOutliersRemoved
			newMeanCircleRadius = np.mean(radiusListOutliersRemoved)
			print "new mean:", newMeanCircleRadius


		#resize template pic so that button is similar size
		if 0.8*newMeanCircleRadius > self.currAvgTemplateRadius or \
			0.8*self.currAvgTemplateRadius > newMeanCircleRadius:
			self.resizeTemplatePics(newMeanCircleRadius)

		# now implement feature detection

		thresh = 0.9
		r = 5000
		# r = int(newMeanCircleRadius) * 10

		buttonNumber = 0

		matchedButtonCentersAndBoxes = []

		for i in range(1, 4):
			button = self.buttonTemplates[i]
			buttonNumber += 1
			buttonCentersAndBoxes = []
			for templatePic in button:

				tempx, tempy = templatePic.shape[:2]

				res = cv2.matchTemplate(img, templatePic, cv2.TM_CCORR_NORMED)

				resdisplay = (res-res.min())/(res.max()-res.min())
				

				while 1:
					minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(res)
					print "stats:", minVal, maxVal, minLoc, maxLoc

					resdisplay1 = (res-res.min())/(res.max()-res.min())
					cv2.imshow("resdisplay1", resdisplay1)
					cv2.waitKey()

					if maxVal < thresh:

						break

					else:

						x,y = maxLoc

						p0 = (x-r, y-r)
						p1 = (x+r, y+r)
						cv2.rectangle(res, p0, p1, (0,0,0), -1) # zero out on res

						pa = (x,y)
						pb = (x+tempx, y+tempy)
						cv2.rectangle(colorimg, pa, pb, (0,0,255), 1)

						#TODO: Might have to do some cleaning here once there is noisy data
						buttonCentersAndBoxes.append(((x, y), pa, pb))

						font = cv2.FONT_HERSHEY_SIMPLEX
						cv2.putText(colorimg, str(buttonNumber), pa, font, 1, (255, 0, 0), thickness = 2)

				cv2.imshow('template', templatePic)
				cv2.waitKey(0)

				cv2.imshow("resdisplay", resdisplay)
				cv2.waitKey(0)

				cv2.imshow("colorimg", colorimg)
				cv2.waitKey(0)

			matchedButtonCentersAndBoxes.append(buttonCentersAndBoxes)

def templateMatchingMethod():

	# btm = buttonTemplateMatching()
	# btm.matchButtonImage()

	ntm = numberTemplateMatching()
	ntm.matchNumberImage()

def main(args):
	# positionBasedLogic()
	# histogramMethod()
	templateMatchingMethod()



if __name__ == '__main__':
	main(sys.argv) 
