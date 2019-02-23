#!/usr/bin/env python

##################################################################################################
#team_id - 5492
#team members - Abhay Sheel Anand, Bhawana Chhaglani, Nikunj Kumar Agarwal and Aniruddha Chauhan
#file name - task3_ip.py
#theme name - pollinator bot
#functions - __init__ and image_callback
##################################################################################################3


import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
import time
class Hey:

	# Function name - __init__
	# Input - None
	# Output - None
	# Logic - subscribes to whycon image out (Constructor function)
	# Example function call - x = Hey()
	def __init__(self):
		# rospy.init_node('ros_bridge')
		self.ros_bridge=cv_bridge.CvBridge()
		print("Bridge Successful")
		# cv2.namedWindow("image", 1)
		self.image_sub=rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
		# self.image_sub=rospy.Subscriber('/whycon/image_out', Image, self.image_callback)
		self.no_of_red=rospy.Publisher('/red', Int32,queue_size=10)

	# Function name - image_callback
	# Input - msg (image)
	# Output - draws contour over the bright spot
	# Logic - detects the bright spot in the image
	# Example function call - image_callback(msg)

	#def get_new_frame(self):
	#	self.image_sub=rospy.Subscriber('/whycon/image_out', Image, self.image_callback)

	def image_callback(self,msg):

		# rate = rospy.Rate(10)
		self.ros_bridge=cv_bridge.CvBridge()
		#self.get_new_frame()
		image = self.ros_bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		
		gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blurred=cv2.GaussianBlur(gray, (11,11),0)
		# cv2.imshow("blurred", blurred)
		# # cv2.waitKey(0)
		# # cv2.destroyAllWindows()
		thresh=cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)[1]
		# # cv2.imshow("thresh", thresh)
		# # cv2.waitKey(0)
		# # cv2.destroyAllWindows()
		im2, contours,hierarchy= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(image, contours, -1, (0,255,0), 3)
		# print("number of bright spots are", len(contours))

		# if len(contours) == 1:
		# 	print("Pollination done! Pollinated 1 Red Daylily Count ")
		self.count_red = len(contours)
		# rate.sleep()
		cv2.imshow("image", image)
		cv2.waitKey(1)
		#cv2.destroyAllWindows()

		self.no_of_red.publish(self.count_red)
        rospy.sleep(0.1)
		# cv_im=image.astype(np.uint8)
		# cv2.imwrite(toString(image), '/home/chetan/Desktop/image')
		# print(image)
		# img=cv2.imread(image,cv2.IMREAD_COLOR)
		# print("image load Successful")
		# cv2.imshow(image,"image")
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
# if __name__=='__main__':
rospy.init_node('test')
test=Hey()
	#cv2.destroyAllWindows()
rospy.spin()
