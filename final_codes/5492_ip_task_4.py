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
		l = [(255,0,0),(0,255,0),(0,0,255)]
		thresh=cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
		# # cv2.imshow("thresh", thresh)
		# # cv2.waitKey(0)
		# # cv2.destroyAllWindows()
		im2, cnts,hierarchy= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		#print("number of bright spots are", len(cnts))
		#cv2.drawContours(image, cnts, -1, (0,0,255), 2)
		self.count_red = len(cnts)
		boxes = []
		for c in cnts:
    			(x, y, w, h) = cv2.boundingRect(c)
    			#leftmost = tuple(c[c[:,:,0].argmin()][0])
    			#rightmost = tuple(c[c[:, :, 0].argmax()][0])
    			#topmost = tuple(c[c[:, :, 1].argmin()][0])
    			#bottommost = tuple(c[c[:, :, 1].argmax()][0])
    			#print (leftmost, rightmost)
    			#colour1 = image[leftmost[1]-1,leftmost[0]-1]
    			#colour2 = image[rightmost[1]+1,rightmost[0]+1]
    			#colour3 = image[topmost[1]-1,topmost[0]-1]
    			#colour4 = image[bottommost[1]+1,bottommost[0]+1]

    			#colour = [(colour1[0]+colour2[0]+colour3[0]+colour4[0])/4, (colour1[1]+colour2[1]+colour3[1]+colour4[1])/4, (colour1[2]+colour2[2]+colour3[2]+colour4[2])/4]
    			
    			colour1 = image[y-3, x-3]
    			colour2 = image[y+h+3, x+w+3]
    			colour = [(int(colour1[0])+int(colour2[0]))/2, (int(colour1[1])+int(colour2[1]))/2, (int(colour1[2])+int(colour2[2]))/2]
    			detect_color = l[np.argmax(colour)]

    			cv2.rectangle(image, (x-3,y-3), (x+w+3,y+h+3), detect_color, 2)

    			
		cv2.imshow("image", image)
		cv2.waitKey(1)
		#cv2.destroyAllWindows()

		self.no_of_red.publish(self.count_red)
        rospy.sleep(0.1)
		
# if __name__=='__main__':
rospy.init_node('test')
test=Hey()
	#cv2.destroyAllWindows()
rospy.spin()