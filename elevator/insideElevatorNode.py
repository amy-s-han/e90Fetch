#!/usr/bin/env python

import roslib
roslib.load_manifest('fetchros')

import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os.path
import rospy
import rospkg
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from fetchros.msg import button
from fetchros.msg import elevatorButtonArray

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(5)
# CONTROL_PERIOD = rospy.Duration(0.01)

# wait time between actions
# WAIT_DURATION = rospy.Duration(0.5)
WAIT_DURATION = rospy.Duration(30)


# Note: Make sure incoming image is less than 1000x1000 in size
# and that the image size matches the img size in pc2


class buttonTemplateMatching:

	def __init__(self, path):
		# load templates of elevator buttons
		self.button1 = []
		self.button2 = []
		self.button3 = []


		self.detectedButtonNumbers = []
		self.path = path

		pathPrefix = path + '/elevator/elevatorTemplatePics/'

		buttonIndex = 0

		while(1):

			workDone = False

			for buttonNumber in range(1,4):

				picPath = pathPrefix + str(buttonNumber) + "button" + str(buttonIndex) + ".jpg"

				if os.path.isfile(picPath): 
					pic = cv2.imread(picPath)
			
					if pic is None:
						print picname, "did not load properly"
						continue

					pic = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)

					if buttonNumber == 1:
						self.button1.append(pic)
					elif buttonNumber == 2:
						self.button2.append(pic)
					else:
						self.button3.append(pic)

					workDone = True


			buttonIndex += 1

			if not workDone:
				break

		self.buttonArray = [self.button1, self.button2, self.button3]
		print "finished loading", len(self.button1) + len(self.button2) + len(self.button3), " templates"

		# debug
		# for i in self.button1:
		# 	cv2.imshow("1", i)
		# 	cv2.waitKey(0)

		# for i in self.button2:
		# 	cv2.imshow("2", i)
		# 	cv2.waitKey(0)

		# for i in self.button3:
		# 	cv2.imshow("3", i)
		# 	cv2.waitKey(0)


	# Note: The resize is for testing purposes. We need the image and
	# the pc2 cloud to match so the template matching must be done on
	# the original image (that has same dimen as pc2 cloud).
	def adjustPicture(self, img):
		img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		h, w = img.shape[:2]

		if h > w:
			rows,cols = img.shape
			M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
			img = cv2.warpAffine(img,M,(cols,rows))

		
		if w > 1000 or h > 1000:
			xmag = 640.0/w
			ymag = 480.0/h

			mag = max(xmag, ymag)

			img = cv2.resize(self.img, (0, 0), fx=mag, fy=mag)

		return img

	def checkCirclesDetected(self, circles):
		colorImg = cv2.cvtColor(self.adjusted, cv2.COLOR_GRAY2BGR)

		if circles is not None:
			circles = np.uint16(np.around(circles))

			for i in circles[0,:]:
				# draw the outer circle
				cv2.circle(colorImg,(i[0],i[1]),i[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(colorImg,(i[0],i[1]),2,(0,0,255),3)

		cv2.imwrite(self.path + "/src/testPicDetectedCircles2.jpg", colorImg)

		cv2.imshow('Detected Circles, press q to quit',colorImg)
		decision = cv2.waitKey(0)
		cv2.destroyAllWindows()

		return decision


	def findAllNumberedButtons(self, debug=False):

		# self.adjusted = cv2.medianBlur(self.adjusted, 5)

		h, w = self.adjusted.shape[:2]
		
		# circles = cv2.HoughCircles(self.adjusted, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=100) 
		circles = cv2.HoughCircles(self.adjusted, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=50, param2=35, maxRadius=20) 


		if self.checkCirclesDetected(circles) == 1048689: # q was pressed, exit
			print "\nExiting.\n"
			return

		bestMatchButtonNumber = []
		bestMatchRes = []
		# buttonImages = []

		templateMatchResult = []

		if circles is None:
			print "No circles detected, returning None"
			return None
		else:
			print len(circles[0,:]), " circles detected"

		for i in circles[0,:]:

			# extract square img that includes circle

			center = (i[0], i[1])
			radius = i[2]

			fudge = 5
			fudgedRadius = radius + fudge

			# draw a bounding box
			p0 = (center[0] - fudgedRadius, center[1] + fudgedRadius)
			p1 = (center[0] + fudgedRadius, center[1] - fudgedRadius)

			buttonImg = img[p1[1]:p0[1], p0[0]:p1[0]]
			
			bImgHeight, bImgWidth = buttonImg.shape[:2]

			if bImgHeight <= 0 or bImgWidth <= 0 or bImgHeight > h or bImgWidth > w:
				print "buttonImg is not valid with w,h = ", (bImgHeight, bImgWidth), ". Skipping."
				continue

			#resize picture to 64x64
			buttonImg = cv2.resize(buttonImg, dsize=(64, 64))

			if debug:
				cv2.imshow('buttonImg',buttonImg)
				if cv2.waitKey(0) == 1048689: # q was pressed
					return

			bestMatchingRes = 0
			bestMatchingButton = None
			
			
			# for each templated button number
			for j in range(3):
				buttonClass = self.buttonArray[j]
				highestRes = 0

				for template in buttonClass:
					res = cv2.matchTemplate(buttonImg, template, cv2.TM_CCORR_NORMED)
					# print res

					# cv2.imshow('template',template)
					# if cv2.waitKey(0) == 1048689: # q was pressed
					# 	return
					if res[0][0] > highestRes:
						highestRes = res[0][0]
				
				if highestRes > bestMatchingRes: #this button number is a better match
					bestMatchingRes = highestRes
					bestMatchingButton = j+1 # this corresponds to the button's number


			if debug:
				print "this image looks like button ", bestMatchingButton, "with res: ", bestMatchingRes

				cv2.imshow('buttonImg, q to return',buttonImg)
				if cv2.waitKey(0) == 1048689: # q was pressed
					return

			# buttonImages.append(buttonImg)
			bestMatchButtonNumber.append(bestMatchingButton)
			bestMatchRes.append(bestMatchingRes)

			templateMatchResult.append([bestMatchingRes, bestMatchingButton, buttonImg, (center, radius)])

		templateMatchResult.sort(key=lambda x: x[0], reverse=True)

		if debug:
			for item in templateMatchResult:
				print "Results: This image looks like button ", item[1], "with res: ", item[0]
				cv2.imshow("image , q to return", item[2])
				if cv2.waitKey(0) == 1048689: # q was pressed
					return
		
		top3 = []
		detectedButtonNumbers = []

		for i in range(len(templateMatchResult)):
			if i > len(templateMatchResult) - 1 or len(top3) == 3:
				break

			buttonNumber = templateMatchResult[i][1]

			# if this button number is already in detectedButtonNumbers, 
			# then this button's res is lower and we don't want it
			if buttonNumber in detectedButtonNumbers:
				continue

			circleInfo = templateMatchResult[i][3]

			top3.append([circleInfo, buttonNumber])
			detectedButtonNumbers.append(buttonNumber)

		return top3


	# searches p, then b, then a. Start search at top
	# left corner, then go around p in a clockwise fashion
	# If we get a non-nan at any point, return that point

	#					a a a a a
	#					a b b b a
	#					a b p b a
	#					a b b b a
	#					a a a a a

	def radialSearch(self, cx, cy, radius):

		point = self.pointCloud[cx, cy]

		# max is the smallest odd number whose squared value
		#  is equal or larger than the diameter
		maxR = np.floor(np.sqrt(2*radius))
		if maxR%2 == 0:
			maxR += 1

		for h in range(0, maxR):

			# h = 0, i = 1
			# h = 1, i = 3
			# h = 2, i = 5
			i = 2*h + 1

			x = cx - h
			y = cy - h

			#first check top row around point
			for j in range(i + 2):
				p = self.pointCloud[x + j, y]
				if p is not nan:
					return p

			x += i + 1

			# now check right vertical around point
			for k in range(1, i + 2):
				p = self.pointCloud[x, y + k]
				if p is not nan:
					return p

			y += i + 1				

			# now check bottom row around point
			for l in range(1, i + 2):
				p = self.pointCloud[x - l, y]
				if p is not nan:
					return p

			x = cx - h

			# now check left vertical around point
			for m in range(1, i + 1):
				p = self.pointCloud[x, y - k]
				if p is not nan:
					return p
		
		print "no point found"
		return None		



	def templateMatching(self, img, pointCloud=None):

		try: 
			self.adjusted = self.adjustPicture(img)
			colorImg = cv2.cvtColor(self.adjusted, cv2.COLOR_GRAY2BGR)
			
			# allButtons = self.findAllNumberedButtons(img, True)
			allButtons = self.findAllNumberedButtons()

			if allButtons is None:
				print "No buttons detected in templateMatching!"
				return None
			else:
				print "All Buttons", allButtons

			msg = elevatorButtonArray()
			msg = []


			for i in allButtons:

				circleInfo = i[0]
				buttonNumber = i[1]

				center = circleInfo[0]
				radius = circleInfo[1]

				buttonData = button()

				print center, radius

				cx = center[0]
				cy = center[1]


				buttonData.buttonNumber = buttonNumber
				buttonData.x = cx
				buttonData.y = cy


				buttonData.radius = radius

				# get the xyz point data from the registered point cloud
				if pointCloud:
					print "Got pc2Data~ need to do this though"

					point = pointCloud[cx, cy]
					
					if point is nan:
						self.pointCloud = pointCloud
						point3d = self.radialSearch(cx, cy, radius)
						if point3d:
							buttonData.x3d = point3d[0]
							buttonData.y3d = point3d[1]
							buttonData.z3d = point3d[2]

					else:
						buttonData.x3d = point[0]
						buttonData.y3d = point[1]
						buttonData.z3d = point[2]


				print "buttonData" , i, ": ", buttonData

				msg.append(buttonData)

				# draw outer circle
				cv2.circle(colorImg, center, radius, (0,255,0), 2)
				# draw the center of the circle
				cv2.circle(colorImg, center, 2, (0,0,255), 3)

				font = cv2.FONT_HERSHEY_SIMPLEX
				cv2.putText(colorImg, str(buttonNumber), (int(center[0] - 0.25*radius), int(center[1] + 0.25*radius)), font, 1, (255, 0, 0), thickness = 2)

			print "msg", msg

			cv2.imwrite(self.path + "/src/testPicDetectedButtons2.jpg", colorImg)

			cv2.imshow("press k to accept msg", colorImg)

			if cv2.waitKey(0) == 1048683: # k was pressed
				return msg
			else:
				return None
				
		except:
			pass




class elevatorButtonDetector:
	def __init__(self):
		print "HERE I AMMMMM"

		rospy.init_node('elevatorButtonDetector', anonymous=True)

		self.buttonPub = rospy.Publisher("/detectedButtons", elevatorButtonArray, queue_size=10)

		rospack = rospkg.RosPack()
		self.packPath = rospack.get_path('fetchros')

		self.bridge = CvBridge()
		
		self.window = 'Camera'

		self.btm = buttonTemplateMatching(self.packPath)

		self.noButtonMsg = elevatorButtonArray()
		button1 = button()
		button1.buttonNumber = -1
		self.noButtonMsg = [button1]

		self.floorButtons = self.noButtonMsg

		# self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.imgCallback)
		self.image_sub = rospy.Subscriber("/head_camera/depth_registered/points", PointCloud2.msg, self.pc2callback)
			
		#fake one for now
		# self.image_sub = rospy.Subscriber("/elevatorImage", Image, self.imgCallback)


		# set up control timer at 100 Hz
		rospy.Timer(CONTROL_PERIOD, self.control_callback)

	def imgCallback(self, data): 
		
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imwrite(self.packPath + "/src/testPic2.jpg", img)

		# print "got image", img.shape[:2]
		cv2.imshow(self.window, img)
		cv2.waitKey(3)

		self.floorButtons = self.btm.templateMatching(img)
		print self.floorButtons

		if self.floorButtons is None:
			self.floorButtons = self.noButtonMsg



	def pc2callback(self, data):
		
		gen = pc2.read_points(data, skip_nans=False)

		int_data = list(gen)

		pointCloud = np.zeros((h, w))
		img = np.zeros((h, w, 3))

		i, j = 0
		
		for x in int_data:
			test = x[3]
			s = struct.pack('>f', test)
			i = struct.unpack('>l', s)[0]

			pack = ctypes.c_uint32(i).value
			r = (pack & 0x00FF0000) >> 16
			g = (pack & 0x0000FF00) >> 8
			b = (pack & 0x000000FF) 

			img[i, j, 0] = b
			img[i, j, 1] = g
			img[i, j, 2] = r

			pointCloud[i, j] = [x[0], x[1], x[2]]

			if j == w:
				j = 0
				i += 1

		imPath = self.packPath + "/src/imgFromPc2.jpg"
		cv2.imwrite(imPath, img)
		
		img2 = cv2.imread(imPath)

		self.floorButtons = self.btm.templateMatching(img2, pointCloud)
		print self.floorButtons

		if self.floorButtons is None:
			self.floorButtons = self.noButtonMsg


	
	def control_callback(self, timer_event=None):

		if self.floorButtons: 
			self.buttonPub.publish(self.floorButtons)
			if self.floorButtons[0].buttonNumber == -1:
				print "No buttons detected"
			else:
				print "Published. ", len(self.floorButtons), " elevator buttons detected."
		else:
			print "Error: floorButtons is None"

		cv2.destroyAllWindows()
		
	def run(self):
		rospy.spin()


def main(args):
	try: 


		ebd = elevatorButtonDetector()
		ebd.run()

	except rospy.ROSInterruptException:
		pass	


if __name__ == '__main__':
	main(sys.argv) 
