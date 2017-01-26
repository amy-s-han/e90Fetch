#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from argparse import ArgumentParser


# I guess I'm prototyping here and if it works, I'll translate it into c++???

class elevatorButtonDetector:
	def __init__(self):
		print "HERE IA MMMMM"
		self.tag_exist_pub = rospy.Publisher("tag_pub", String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)

		self.window = 'Camera'
		cv2.namedWindow(self.window)

	def callback(self,data): 
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imshow(self.window, cv_image)
		cv2.waitKey(3)

		try:
			
			gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
			
			#elevator logic here

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