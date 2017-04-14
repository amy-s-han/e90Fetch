#!/usr/bin/env python

import roslib
roslib.load_manifest('fetchros')

import sys
import rospy
import cv2
import cv2.cv as cv
import rospkg
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import os.path
import ctypes
import struct

from fetchros.msg import button
from fetchros.msg import elevatorButtonArray


class registeredPoints:
	def __init__(self):
		print "here once"
		rospy.init_node('getRegPoints', anonymous=True)

		rospack = rospkg.RosPack()
		self.packPath = rospack.get_path('fetchros')

		self.pc2_sub = rospy.Subscriber("/head_camera/depth_registered/points", PointCloud2, self.pc2callback)

		self.gotCloud = False
		self.data = None

	def pc2callback(self, data):

		if self.gotCloud:
			print "got cloud"
			return

		
		self.data = data
		if self.data is not None:
			self.gotCloud = True
		
		
		# write pc2 to file

		gen = pc2.read_points(data, skip_nans=False)

		int_data = list(gen)
		

		f = open(self.packPath + "/src/pc2JustBoard2.txt", "w")
		points = open(self.packPath + "/src/pointsFromPc2JustBoard2.txt", "w")
		colors = open(self.packPath + "/src/colorsFromPc2JustBoard2.txt", "w")

		for x in int_data:
			test = x[3]
			s = struct.pack('>f', test)
			i = struct.unpack('>l', s)[0]

			pack = ctypes.c_uint32(i).value
			r = (pack & 0x00FF0000) >> 16
			g = (pack & 0x0000FF00) >> 8
			b = (pack & 0x000000FF) 

			#print x[0], x[1], x[2], r, g, b
			a = [x[0], x[1], x[2], r, g, b]
			for i in a:

				f.write(str(i) + " ")
			
			points.write(str(x[0]) + " " + str(x[1]) + " " + str(x[2]) + "\n")

			colors.write(str(r) + " " + str(g) + " " + str (b) + "\n")
			
			f.write("\n")
		
		f.close()
		points.close()
		colors.close()
			


def main(args):
	try:
		registeredPoints()
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass	


if __name__ == '__main__':
	main(sys.argv) 
