#!/usr/bin/env python
from plutodrone.srv import *
import rospy
from std_msgs.msg import Float64,Int16

class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_board_data')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		self.pub_yaw = rospy.Publisher('/publish_yaw_apna',Float64, queue_size=1)
		rospy.spin()

	def access_data(self, req):
		 print "accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ)
		 print "gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ)
		 print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		 print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		 print "altitude = " +str(req.alt)
		 self.pub_yaw.publish(req.yaw)
		 rospy.sleep(.1)
		 return PlutoPilotResponse(rcAUX2 =1500)

test = request_data()