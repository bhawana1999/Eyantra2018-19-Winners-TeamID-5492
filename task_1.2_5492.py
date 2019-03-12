#!/usr/bin/env python
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import *
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import rospy
import time


class DroneFly():
    """docstring for DroneFly"""

    def __init__(self):

        #rospy.init_node('pluto_fly', disable_signals=True)
        rospy.init_node('ros_bridge', disable_signals=True)

        self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

	#rospy.init_node('Pluto_fly', disable_signals=True)

        rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

        #function that subscribes to the topic /drone_yaw
        rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)

        # To tune the drone during runtime
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)

        self.cmd = PlutoMsg()

        

	#Thresholds to Hold
    	self.thresh=0.2
    	self.threshZ=0.5

    	#Points to go to
    	w, h = 3,5 
    	self.points = [[0.0 for x in range(w)] for y in range(h)] 
    	self.points[0]=[-5.63,-5.63,30]
    	self.points[1]=[5.57,-5.63,30]
    	self.points[2]=[5.55,5.54,30]
    	self.points[3]=[-5.6,5.54,30]
	
	#Initial Position to hold.
        self.wp_x = self.points[0][0]
        self.wp_y = self.points[0][1]
        self.wp_z = self.points[0][2]

        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        self.cmd.plutoIndex = 0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_yaw = 0.0

        # PID constants for Roll
        self.kp_roll = 7.0  
        self.ki_roll = 0.0
        self.kd_roll = 0.0 

        # PID constants for Pitch
        self.kp_pitch = 8.0  
        self.ki_pitch = 0.0
        self.kd_pitch = 0.0

        # PID constants for Yaw
        self.kp_yaw = 15.0
        self.ki_yaw = 0.0
        self.kd_yaw = 4.0

        # PID constants for Throttle
        self.kp_throt = 32.0  
        self.ki_throt = 3.0   
        self.kd_throt = 25.0  

        # Correction values after PID is computed
        self.correct_roll = 0.0
        self.correct_pitch = 0.0
        self.correct_yaw = 0.0
        self.correct_throt = 0.0

        # Loop time for PID computation. You are free to experiment with this
        self.last_time = 0.0
        self.loop_time = 0.060

        # self Defined variables

        self.errorsumX = 0
        self.errorsumY = 0
        self.errorsumZ = 0
        self.errorsumYaw = 0

        self.prevErrorY = 0
        self.prevErrorX = 0
        self.prevErrorZ = 0
        self.prevErrorYaw = 0

        rospy.sleep(.1)


    def get_no_of_colour(self):

        self.ros_bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('whycon/image_out', Image, self.image_callback)

        self.no_of_red=rospy.Publisher('/red', Int32,queue_size=10)
        self.no_of_blue=rospy.Publisher('/blue', Int32,queue_size=10)
        self.no_of_green=rospy.Publisher('/green', Int32,queue_size=10)

    def image_callback(self, msg):
        image1 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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

        self.no_of_blue.publish(self.count_blue)
        rospy.sleep(0.1)

        self.no_of_red.publish(self.count_red)
        rospy.sleep(0.1)

        # print count_blue
        # print count_green
        # print count_red


    def arm(self):
        self.cmd.rcAUX4 = 1500
        self.cmd.rcThrottle = 1000
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    def position_hold(self):
        # self.get_no_of_colour()
        rospy.sleep(2)

        print "disarm"
        self.disarm()
        rospy.sleep(.2)
        print "arm"
        self.arm()
        rospy.sleep(.1)

        #variable that stores initial value(setpoint) of yaw

        self.init_yaw = self.drone_yaw
        self.counter = 0
        while True:
            self.calc_pid()
            # print(abs(self.drone_x-self.wp_x),abs(self.drone_y-self.wp_y),abs(self.drone_z-self.wp_z))
            if self.counter<4 and abs(self.drone_x-self.wp_x)<self.thresh  and abs(self.drone_y-self.wp_y)<self.thresh and abs(self.drone_z-self.wp_z)<self.threshZ:
                self.counter=self.counter+1
                print("reached point", self.counter)
                self.wp_x = self.points[self.counter][0]
                self.wp_y = self.points[self.counter][1]
                self.wp_z = self.points[self.counter][2]

            # Check your X and Y axis. You MAY have to change the + and the -.
            # We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
            pitch_value = int(1500 - self.correct_pitch)
            self.cmd.rcPitch = self.limit(pitch_value, 1600, 1400)

            roll_value = int(1500 - self.correct_roll)
            self.cmd.rcRoll = self.limit(roll_value, 1600, 1400)

            throt_value = int(1400 - self.correct_throt)
            self.cmd.rcThrottle = self.limit(throt_value, 1550, 1250)

            yaw_value = int(1500 + self.correct_yaw)
            self.cmd.rcYaw = self.limit(yaw_value, 1570, 1420)

            self.pluto_cmd.publish(self.cmd)

    def calc_pid(self):
        self.seconds = time.time()
        current_time = self.seconds - self.last_time
        if (current_time >= self.loop_time):
            self.pid_throt()
            self.pid_yaw()
    	    self.pid_pitch()
            self.pid_roll()
            
            self.last_time = self.seconds

    def pid_roll(self):
        error = self.wp_y - self.drone_y
        self.errorsumY += error
        dErr = error - self.prevErrorY
        PID = self.kp_roll * error + self.kd_roll * dErr / self.loop_time + self.ki_roll * self.errorsumY * self.loop_time
        self.prevErrorY = error
        self.correct_roll = PID


    def pid_pitch(self):
        # Compute Pitch PID here
        error = self.wp_x - self.drone_x
        self.errorsumX += error
        dErr = error - self.prevErrorX
        PID = self.kp_pitch * error + self.kd_pitch * dErr / self.loop_time + self.ki_pitch * self.errorsumX * self.loop_time
        self.correct_pitch = PID


    def pid_throt(self):
        # Compute Throttle PID here
        error = self.wp_z - self.drone_z
        self.errorsumZ += error
        dErr = error - self.prevErrorZ
        PID = self.kp_throt * error + self.kd_throt * dErr / self.loop_time + self.ki_throt * self.errorsumZ * self.loop_time
        self.prevErrorZ = error
        self.correct_throt = PID


    def pid_yaw(self):
        # yaw tuning
        error = self.init_yaw - self.drone_yaw
        self.errorsumYaw += error
        dErr = error - self.prevErrorYaw
        PID = self.kp_yaw * error + self.kd_yaw * dErr / self.loop_time + self.ki_yaw * self.errorsumYaw * self.loop_time
        self.prevErrorYaw = error
        self.correct_yaw = PID


    def limit(self, input_value, max_value, min_value):

        # Use this function to limit the maximum and minimum values you send to your drone

        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value

    # You can use this function to publish different information for your plots
    #\ def publish_plot_data(self):

    def set_pid_alt(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

        self.kp_throt = pid_val.Kp
        self.ki_throt = pid_val.Ki
        self.kd_throt = pid_val.Kd

    def set_pid_roll(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

        self.kp_roll = pid_val.Kp
        self.ki_roll = pid_val.Ki
        self.kd_roll = pid_val.Kd

    def set_pid_pitch(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

        self.kp_pitch = pid_val.Kp
        self.ki_pitch = pid_val.Ki
        self.kd_pitch = pid_val.Kd

    def set_pid_yaw(self, pid_val):

        # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

        self.kp_yaw = pid_val.Kp
        self.ki_yaw = pid_val.Ki
        self.kd_yaw = pid_val.Kd

    def get_pose(self, pose):

        # This is the subscriber function to get the whycon poses
        # The x, y and z values are stored within the drone_x, drone_y and the drone_z variables

        self.drone_x = pose.poses[0].position.x
        self.drone_y = pose.poses[0].position.y
        self.drone_z = pose.poses[0].position.z

    #subscriber function to get data from the topic /drone_yaw
    def get_yaw(self, data):
        self.drone_yaw = data.data


if __name__ == '__main__':
    while not rospy.is_shutdown():
        temp = DroneFly()
        temp.get_no_of_colour()
        temp.position_hold()
        rospy.spin()
