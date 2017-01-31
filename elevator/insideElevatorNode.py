#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from argparse import ArgumentParser


# I guess I'm prototyping here and if it works, I'll translate it into c++???

def detectButton(img):
	try:

		#img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3) #not sure if needed

		img = cv2.medianBlur(img,5)
                print "here"
		cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
                print "but not here"
		circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=75, param2=71) 
                print "got here"

		circles = np.uint16(np.around(circles))
		toSortCircles = np.int16(np.around(circles))

		# draw all circles
		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
			print i

		sortedCircle = toSortCircles[0]
		# print "before sorting", sortedCircle

		sortedCircle = sortedCircle[sortedCircle[:,1].argsort()]
		print "after sorting", sortedCircle

		candidates = []
		numCircles = sortedCircle.shape[0]

		for i in range(1, numCircles):

			# if y for this circle is far away from y of last circle,
			# that means that this is a new row of circles and this might 
			# be a candidate
			if abs(sortedCircle[i][1] - sortedCircle[i-1][1]) > 0.1 * sortedCircle[i-1][1]:
				# check if this circle has a neighbour that is on the same row.
				# if no neighbour on the same row, then add to candidates

				if abs(sortedCircle[i][1] - sortedCircle[i+1][1]) > 0.1 * sortedCircle[i-1][1]:
					candidates.append(sortedCircle[i])

			# if this is the last circle: 
			if i == numCircles - 1:
				#check to see if this circle is a candidate
				if len(candidates) != 0:
					if abs(sortedCircle[i][0] - candidates[0][0]) < 0.1 * candidates[0][0]:
						candidates.append(sortedCircle[i])
			
		print candidates

		badIndices = []

		# print "\n\n here \n\n"

		# run through candidates to make sure they have similar x 
		for i in range(1, len(candidates)):
			# check that the buttons are vertically aligned
			if abs(candidates[i][0] - candidates[i-1][0]) > 0.05 * candidates[i-1][0]:
				badIndices.append(i)

		# print badIndices

		if badIndices != []:
			for bad in badIndices:
				candidates.remove(bad)

		# print candidates

		# draw out the elevator floor buttons
		for circ in candidates:
			cv2.circle(cimg, (circ[0], circ[1]), circ[2], (255, 0, 0), 3)

		cv2.imshow('detected circles',cimg)
		return candidates

	except:
		pass

class elevatorButtonDetector:

	def __init__(self):
		print "HERE I AMMMMM"
		self.button_exist_pub = rospy.Publisher("button_pub", String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.callback)

		self.window = 'Camera'
	

	def callback(self,data): 
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imshow(self.window, img)
		cv2.waitKey(3)

		floorButtons = detectButton(img)
		if floorButtons: 
			self.button_exist_pub.publish(floorButtons)
			print "Published. ", floorButtons.len, " elevator buttons detected."
		else:
			print "no elevator buttons detected"
			

def main(args):
	tag = elevatorButtonDetector()
	rospy.init_node('elevatorButtonDetector', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv) 
