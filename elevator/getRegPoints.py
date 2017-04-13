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
import os.path

from fetchros.msg import button
from fetchros.msg import elevatorButtonArray


class registeredPoints:
	def __init__(self):

		rospy.init_node('getRegPoints', anonymous=True)

		rospack = rospkg.RosPack()
		self.packPath = rospack.get_path('fetchros')

		self.image_sub = rospy.Subscriber("/head_camera/depth_registered/points", PointCloud2.msg, self.pc2callback)

	def pc2callback(self, data):

		print data

		# write pc2 to file

		f = open(self.packPath + "pc2.txt", "w")

		for i in data:
			f.write(i)



def main(args):
	try:
		registeredPoints()
	except rospy.ROSInterruptException:
		pass	


if __name__ == '__main__':
	main(sys.argv) 