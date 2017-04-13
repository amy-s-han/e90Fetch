#!/usr/bin/env python

import roslib
roslib.load_manifest('fetchros')

import sys
import rospy
import cv2
import cv2.cv as cv
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from fetchros.msg import button
from fetchros.msg import elevatorButtonArray


# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# wait time between actions
WAIT_DURATION = rospy.Duration(0.5)

class testNode:
	def __init__(self):
		print "HERE I AMMMMM"

		rospy.init_node('testNode', anonymous=True)


		self.imgPub = rospy.Publisher("/elevatorImage", Image, queue_size=10)


		# get an instance of RosPack with the default search paths
		rospack = rospkg.RosPack()

		# get the file path for rospy_tutorials
		packPath = rospack.get_path('fetchros')

		print packPath

		self.img = cv2.imread(packPath + '/src/elevatorPics/inside1.jpg')

		if self.img is None:
			print "uhhhh image did not load"
			return
		else:
			print "Image loaded"
			
			w, h = self.img.shape[:2]

			if w > 1000 or h > 1000:
				xmag = 950.0/w
				ymag = 950.0/h

				mag = max(xmag, ymag)


				self.img = cv2.resize(self.img, (0, 0), fx=mag, fy=mag)
			
			w, h = self.img.shape[:2]

			print w, h



		self.bridge = CvBridge()

		rospy.Timer(CONTROL_PERIOD, self.control_callback)


	def control_callback(self, timer_event=None):
		try:
			self.imgPub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
			cv2.imshow("img", self.img)
			cv2.waitKey(3)

		except CvBridgeError as e:
			print "ERROR"
			print(e)

	def run(self):
		rospy.spin()

def main(args):
	try:
		t = testNode()
		t.run()
	except rospy.ROSInterruptException:
		pass	


if __name__ == '__main__':
	main(sys.argv) 