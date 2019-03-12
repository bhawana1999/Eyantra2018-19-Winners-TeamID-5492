#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

class WayPoint:
	
	def __init__(self):

		rospy.init_node('ros_bridge')
		
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()

		#Pubish No. of
		# self.no_red = rospy.Publisher('/red', int , queue_size = 10)
		# self.no_green = rospy.Publisher('/green', int , queue_size = 10)
		# self.no_blue = rospy.Publisher('/blue', int , queue_size = 10)
		

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('whycon/image_out', Image, self.image_callback)
		
	def image_callback(self,msg):

		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
		image1 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		kernel = np.ones((5,5), np.uint8)
		image = cv2.erode(image1, kernel , iterations=1)
		count_blue = 0
		count_red = 0
		count_green = 0

		l1 = np.array([100,0,0])
		u1 = np.array([255,0,0])
		mask_blue = cv2.inRange(image,l1,u1)
		blue = cv2.bitwise_and(image, image, mask=mask_blue)
		gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
		ret,thresh = cv2.threshold(gray,15,255,0)
		im2,contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		count_blue = len(contours)

		l2 = np.array([0,200,0])
		u2 = np.array([0,220,0])
		mask_green = cv2.inRange(image,l2,u2)
		green = cv2.bitwise_and(image, image, mask= mask_green)
		gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
		ret,thresh = cv2.threshold(gray,15,255,0)
		im2,contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		count_green = len(contours)

		l3 = np.array([0,0,100])
		u3 = np.array([0,0,255])
		mask_red = cv2.inRange(image,l3,u3)
		red = cv2.bitwise_and(image, image, mask= mask_red)
		gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
		ret,thresh = cv2.threshold(gray,15,255,0)
		im2,contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		count_red = len(contours)
		print count_blue
		print count_green
		print count_red

		
if __name__ == '__main__':
	test = WayPoint()
	rospy.spin()

		