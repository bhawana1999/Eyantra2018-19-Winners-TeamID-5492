#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class WayPoint:
	
	def __init__(self):

		# rospy.init_node('ros_bridge')
		rospy.init_node('ros_bridge', disable_signals=True)
		
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('/whycon/image_out', Image, self.image_callback)
		# self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback, queue_size =1)
	def image_callback(self, msg):
		image1 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		cv2.imshow('image1',image1)
		kernel = np.ones((5,5), np.uint8)
		image = cv2.erode(image1, kernel , iterations=1)
		self.count_blue = 0
		self.count_red = 0
		self.count_green = 0

		l1 = np.array([100,0,0])
        u1 = np.array([255,0,0])
        mask_blue = cv2.inRange(image,l1,u1)
        blue = cv2.bitwise_and(image, image, mask=mask_blue)
        gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,15,255,0)
        im2,contours_blue,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        self.count_blue = len(contours_blue)

        l2 = np.array([0,200,0])
        u2 = np.array([0,220,0])
        mask_green = cv2.inRange(image,l2,u2)
        green = cv2.bitwise_and(image, image, mask= mask_green)
        gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,15,255,0)
        im2,contours_green,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        self.count_green = len(contours_green)

        l3 = np.array([0,0,100])
        u3 = np.array([0,0,255])
        mask_red = cv2.inRange(image,l3,u3)
        red = cv2.bitwise_and(image, image, mask= mask_red)
        gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,15,255,0)
        im2,contours_red,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        self.count_red = len(contours_red)

        self.no_of_green.publish(self.count_green)
        rospy.sleep(0.1)
        image1 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.no_of_blue.publish(self.count_blue)
        rospy.sleep(0.1)

        self.no_of_red.publish(self.count_red)
        rospy.sleep(0.1)		
	# def image_callback(self,msg):
	# 	image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	# 	# image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	# 	cv2.imshow("im",image)
	# 	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# 	blurred = cv2.GaussianBlur(gray, (11, 11), 0)
	# 	cv2.imshow("blurred", blurred)
	# 	cv2.waitKey(3)
	# 	# thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)
	# 	thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)[1]
	# 	# print image.shape
	# 	# thresh = np.random.randint(2, size = image.shape)
	# 	cv2.imshow("thresh", thresh)
 #        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 #        cv2.drawContours(image, contours, -1, (255,0,0), 2)
 #        print ("number of bright spots are" , len(contours))
 #        cv2.imshow("Image", image)
 #        cv2.waitKey(0)
 #        cv2.destroyAllWindows()


		
if __name__ == '__main__':
	test = WayPoint()
	rospy.spin()