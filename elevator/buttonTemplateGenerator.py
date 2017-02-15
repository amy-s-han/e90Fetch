import cv2
import cv2.cv as cv
import numpy as np
import sys
import os.path


button1Count = 0
button2Count = 0
button3Count = 0


def findCircles(img):

	img = cv2.medianBlur(img,5)
	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	rows,cols = img.shape
	M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
	img = cv2.warpAffine(img,M,(cols,rows))

	img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)

	print "after adjustment shape: ", img.shape[:2]

	circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=180) 
	# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=100) 

	# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=80, param2=75) 

	return img, circles

def checkCirclesDetected(grayImg, circles):
	colorImg = cv2.cvtColor(grayImg,cv2.COLOR_GRAY2BGR)

	if circles is not None:
		circles = np.uint16(np.around(circles))

		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(colorImg,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(colorImg,(i[0],i[1]),2,(0,0,255),3)

	cv2.imshow('Detected Circles, press q to quit',colorImg)
	decision = cv2.waitKey(0)
	cv2.destroyAllWindows()

	print "waitkey: ", decision

	return decision

def genButtonPics(grayImg, circles):

	global button1Count
	global button2Count
	global button3Count

	colorImg = cv2.cvtColor(grayImg,cv2.COLOR_GRAY2BGR)

	circles = np.uint16(np.around(circles))

	for i in circles[0,:]:
		center = (i[0], i[1])
		radius = i[2]

		fudge = 5
		fudgedRadius = radius + fudge

		# draw a bounding box
		p0 = (center[0] - fudgedRadius, center[1] + fudgedRadius)
		p1 = (center[0] + fudgedRadius, center[1] - fudgedRadius)

		print p0, p1

		buttonImg = colorImg[p1[1]:p0[1], p0[0]:p1[0]]

		#resize picture to 64x64
		buttonImg = cv2.resize(buttonImg, dsize=(64, 64))

		cv2.imshow('buttonImg',buttonImg)
		decision = cv2.waitKey(0)
		print "waitkey: ", decision

		buttonName = ""
		count = 0
		if decision == 1048625: # if the key '1' is pressed
			buttonName = "1button"
			count = button1Count
		elif decision == 1048626: # if the key '2' is pressed
			buttonName = "2button"
			count = button2Count
		elif decision == 1048627: # if the key '3' is pressed
			buttonName = "3button"
			count = button3Count
		else: # if any other key is pressed, continue
			continue

		print "button name: ", buttonName


		grayButton = cv2.cvtColor(buttonImg, cv2.COLOR_BGR2GRAY)

		pathPrefix = "templatePics/"
		


		while(1):
			temp = pathPrefix + buttonName + str(count) + ".jpg"
			print temp

			if os.path.isfile(temp): 
			# keep incrementing count until the file name doesn't exist
				print "file exists"
				count += 1
			else:
				print "writing to file: ", temp
				cv2.imwrite(temp, grayButton)

				# update counts for buttons
				if decision == 1048625:
					button1Count = count
				elif decision == 1048626:
					button2Count = count
				elif decision == 1048627:
					button3Count = count

				break

		
		# draw the outer circle
		cv2.circle(colorImg,center,radius,(0,255,0),2)
		# draw the center of the circle
		cv2.circle(colorImg,center,2,(0,0,255),3)



def main(args):

	os.system('clear')
	print "If no picture is provided on command line, read in pics/inside1.jpg."
	print "For each circle detected, program will show a resized circle image."
	print "If the image should be saved, enter a number {1, 2, or 3} that corresponds"
	print "to the button number. The image will be saved in the format button+number+count.jpg"
	print "Otherwise, press any other key to continue."

	global button1Count
	global button2Count
	global button3Count

	img = None

	if len(args) == 1:
		# Load static image for testing
		img = cv2.imread('pics/inside1.jpg')
	else:
		# check image exists and load
		if os.path.isfile(args[1]):
			print "loading file: ", args[1]
			img = cv2.imread(args[1])
		else: 
			print args[1], "does not exist. Exiting"
			return

	countFile = None
	if os.path.isfile("templatePics/buttonCount.txt"):
		countFile = open("templatePics/buttonCount.txt", 'rw')
		i = 0

		for line in countFile:

			try:
				count = int(line.strip())
				if i == 0:
					button1Count = count
				elif i == 1:
					button2Count = count
				elif i == 2:
					button3Count = count

			except:
				print "something bad happened during count loading"
				pass

			i += 1

		countFile.close()


	print "loaded button counts: ", button1Count, button2Count, button3Count
	
	grayImg, circles = findCircles(img)

	if checkCirclesDetected(grayImg, circles) == 1048689: # q was pressed, exit
		print "\nExiting.\n"
		return

	if circles is not None:
		genButtonPics(grayImg, circles)


	#write updated counts to file
	countFile = open("templatePics/buttonCount.txt", 'w')

	if countFile is None:
		print "ACKKKKKKK"
		asdf

	countFile.write(str(button1Count) + "\n")					
	countFile.write(str(button2Count) + "\n")					
	countFile.write(str(button3Count) + "\n")	

	countFile.close()


if __name__ == '__main__':
	main(sys.argv) 