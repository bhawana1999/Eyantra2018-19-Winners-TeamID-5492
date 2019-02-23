#team_id - 5492
#team members - Abhay Sheel Anand, Bhawana Chhaglani, Nikunj Kumar Agarwal and Aniruddha Chauhan
#file name - task3_ip.py
#theme name - pollinator bot
#functions - __init__ and image_callback

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
class Hey:

	# Function name - __init__
	# Input - None
	# Output - None
	# Logic - subscribes to whycon image out (Constructor function)
	# Example function call - x = Hey()
	def __init__(self):
		rospy.init_node('ros_bridge')
		self.ros_bridge=cv_bridge.CvBridge()
		print("Bridge Successful")
		self.image_sub=rospy.Subscriber('/whycon/image_out', Image, self.image_callback)

	# Function name - image_callback
	# Input - msg (image)
	# Output - draws contour over the bright spot
	# Logic - detects the bright spot in the image
	# Example function call - image_callback(msg)
	def image_callback(self,msg):
		image=self.ros_bridge.imgsmsg_to_cv2(msg,desired_encoding='bgr8')
		gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blurred=cv2.GaussianBlur(gray, (11,11),0)
		# cv2.imshow("blurred", blurred)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		thresh=cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)[1]
		# cv2.imshow("thresh", thresh)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		im2, cnts,hierarchy= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		boxes = []
		for c in cnts:
    			(x, y, w, h) = cv2.boundingRect(c)
    			boxes.append([x,y, x+w,y+h])

		boxes = np.asarray(boxes)
		# need an extra "min/max" for contours outside the frame
		left = np.min(boxes[:,0])
		top = np.min(boxes[:,1])
		right = np.max(boxes[:,2])
		bottom = np.max(boxes[:,3])

		cv2.rectangle(image, (left,top), (right,bottom), (255, 0, 0), 2)

		#cv2.drawContours(image, contours, -1, (255, 0, 0), 2)
		print("number of bright spots are", len(cnts))
		cv2.imshow("image", image)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		# cv_im=image.astype(np.uint8)
		# cv2.imwrite(toString(image), '/home/chetan/Desktop/image')
		# print(image)
		# img=cv2.imread(image,cv2.IMREAD_COLOR)
		# print("image load Successful")
		# cv2.imshow(image,"image")
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
if __name__=='__main__':
	test=Hey()
	rospy.spin()
