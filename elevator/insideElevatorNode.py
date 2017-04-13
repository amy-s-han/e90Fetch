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

from fetchros.msg import button
from fetchros.msg import elevatorButtonArray

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(5)
# CONTROL_PERIOD = rospy.Duration(0.01)

# wait time between actions
# WAIT_DURATION = rospy.Duration(0.5)
WAIT_DURATION = rospy.Duration(30)



class buttonTemplateMatching:

	def __init__(self, path):
		# load templates of elevator buttons
		self.button1 = []
		self.button2 = []
		self.button3 = []

		pathPrefix = path + '/src/elevatorTemplatePics/'

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

		print "finished loading templates"

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

	def adjustPicture(self, img):
		img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		w,h = img.shape[:2]
		

		if h > w:
			rows,cols = img.shape
			M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
			img = cv2.warpAffine(img,M,(cols,rows))


		if w > 700 or h > 700:
			xmag = 640.0/w
			ymag = 480.0/h

			mag = max(xmag, ymag)

			img = cv2.resize(self.img, (0, 0), fx=mag, fy=mag)

		return img

	def checkCirclesDetected(self, grayImg, circles):
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

		return decision

	def findAllNumberedButtons(self, img, debug=False):

		img = cv2.medianBlur(img,5)
		img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		w,h = img.shape[:2]

		if h > w:
			rows,cols = img.shape
			M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
			img = cv2.warpAffine(img,M,(cols,rows))

		if w > 700 or h > 700:
			xmag = 640.0/w
			ymag = 480.0/h

			mag = max(xmag, ymag)
			img = cv2.resize(img, (0, 0), fx=mag, fy=mag)

		circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=60, maxRadius=100) 

		if self.checkCirclesDetected(img, circles) == 1048689: # q was pressed, exit
			print "\nExiting.\n"
			return

		bestMatchButtonNumber = []
		bestMatchRes = []
		buttonImages = []

		test = []

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
			
			buttonClasses = [self.button1, self.button2, self.button3]

			for i in range(3):
				buttonClass = buttonClasses[i]
				highestRes = 0

				for template in buttonClass:
					res = cv2.matchTemplate(buttonImg, template, cv2.TM_CCORR_NORMED)
					# print res

					# cv2.imshow('template',template)
					# if cv2.waitKey(0) == 1048689: # q was pressed
					# 	return

					if res[0][0] > highestRes:
						highestRes = res[0][0]
				
				if highestRes > bestMatchingRes:
					bestMatchingRes = highestRes
					bestMatchingButton = i+1

			if debug:
				print "this image looks like button ", bestMatchingButton, "with res: ", bestMatchingRes

				cv2.imshow('buttonImg',buttonImg)
				if cv2.waitKey(0) == 1048689: # q was pressed
					return

			buttonImages.append(buttonImg)
			bestMatchButtonNumber.append(bestMatchingButton)
			bestMatchRes.append(bestMatchingRes)

			test.append([bestMatchingRes, bestMatchingButton, buttonImg, (center, radius)])

		test.sort(key=lambda x: x[0], reverse=True)

		if debug:
			for item in test:
				print "Results: This image looks like button ", item[1], "with res: ", item[0]
				cv2.imshow("image", item[2])
				if cv2.waitKey(0) == 1048689: # q was pressed
					return
		
		top3 = []

		top3Range = min(3, len(test))
		for i in range(top3Range):
			print "res: ", test[i][0]
			circleInfo = test[i][3]
			buttonNumber = test[i][1]

			top3.append([circleInfo, buttonNumber])

		return top3

	def templateMatching(self, img):
		try: 
			scaled = self.adjustPicture(img)
			print "here"
			colorImg = cv2.cvtColor(scaled,cv2.COLOR_GRAY2BGR)


			# allButtons = self.findAllNumberedButtons(img, True)
			allButtons = self.findAllNumberedButtons(img)

			if allButtons is None:
				print "No buttons detected in templateMatching!"
				return None

			msg = elevatorButtonArray()
			msg = []

			for i in allButtons:

				circleInfo = i[0]
				buttonNumber = i[1]

				center = circleInfo[0]
				radius = circleInfo[1]

				buttonData = button()
				buttonData.buttonNumber = buttonNumber
				buttonData.x = center[0]
				buttonData.y = center[1]
				buttonData.z = center[2]
				buttonData.radius = radius

				msg.append(buttonData)

				# draw outer circle
				cv2.circle(colorImg, center, radius, (0,255,0), 2)
				# draw the center of the circle
				cv2.circle(colorImg, center, 2, (0,0,255), 3)

				font = cv2.FONT_HERSHEY_SIMPLEX
				cv2.putText(colorImg, str(buttonNumber), (int(center[0] - 0.25*radius), int(center[1] + 0.25*radius)), font, 1, (255, 0, 0), thickness = 2)

			cv2.imshow("img", colorImg)
			if cv2.waitKey(0) == 1048689: # q was pressed
				return msg
				
		except:
			pass




class elevatorButtonDetector:
	def __init__(self):
		print "HERE I AMMMMM"

		rospy.init_node('elevatorButtonDetector', anonymous=True)


		self.buttonPub = rospy.Publisher("/detectedButtons", elevatorButtonArray, queue_size=10)


		# get an instance of RosPack with the default search paths
		rospack = rospkg.RosPack()

		# get the file path for rospy_tutorials
		self.packPath = rospack.get_path('fetchros')

		print self.packPath

		self.bridge = CvBridge()

		# self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.callback)
		
		#fake one for now
		self.image_sub = rospy.Subscriber("/elevatorImage", Image, self.imgCallback)

		self.window = 'Camera'

		self.btm = buttonTemplateMatching(self.packPath)

		self.noButtonMsg = elevatorButtonArray()
		button1 = button()
		button1.buttonNumber = -1

		self.noButtonMsg = [button1]

		self.floorButtons = self.noButtonMsg

		# set up control timer at 100 Hz
		rospy.Timer(CONTROL_PERIOD, self.control_callback)

	def imgCallback(self,data): 

		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# print "got image"
		cv2.imshow(self.window, img)
		cv2.waitKey(3)

		self.floorButtons = self.btm.templateMatching(img)
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