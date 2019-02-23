#!/usr/bin/env python
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import *
import rospy
import time

################################################################################
#Team Id:eyrc#5492                                                             #
#Author List:Bhawana Chhaglani, Abhaysheel Anand                               #
#            Aniruddha Chauhan, Nikunj kumar Agarwal                           #
#Filename:eyrc#5492_pos_hold.py                                                #
#Functions:Arm, Disarm, calc_pid ,position_hold, pid_roll, pid_pitch,          #
#          pid_yaw, pid_throt,set_pid_pitch,set_pid_roll,set_pid_yaw,          #
#          set_pid_throt,limit,get_pos.                                        #
#Global variables:wp_x,wp_y,wp_z,rcRoll,rcPitch,rcYaw,rcThrottle,drone_x,      #
#                 drone_y,drone_z,drone_yaw,kp_roll,ki_roll,kd_roll,           #
#                 ki_pitch ,kd_pitch,ki_pitch,ki_yaw,kp_yaw,kd_yaw,ki_throt,   #
#                 kp_throt,kd_throt.                                           #
################################################################################
class DroneFly():
    """docstring for DroneFly"""

    def __init__(self):

        rospy.init_node('pluto_fly', disable_signals=True)

        self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

        rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

        #function that subscribes to the topic /drone_yaw in VREEEEPPPPPPPP
        # rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)

        # To tune the drone during runtime
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
        rospy.Subscriber('/input_key',Int16, self.our_disarm)
        rospy.Subscriber('/publish_yaw_apna',Float64, self.get_yaw)
        self.cmd = PlutoMsg()

        # Position to hold.
        self.wp_x = -5.4
        self.wp_y = 0.45
        self.wp_z = 19.3
        #variable that stores initial value(setpoint) of yaw

        self.init_yaw = 0

        # self.cmd.rcRoll = 1500
        # self.cmd.rcPitch = 1500
        # self.cmd.rcYaw = 1500
        # self.cmd.rcThrottle = 1500
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        # self.cmd.rcAUX4 = 1000
        self.cmd.rcAUX4 = 1500
        self.cmd.plutoIndex = 0

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_yaw = 0.0 #change

        # PID constants for Roll
        self.kp_roll = 4.5
        self.ki_roll = 0
        # self.kd_roll = 4.2
        self.kd_roll = 5
        # self.kd_roll = 1.0

        # PID constants for Pitch
        self.kp_pitch = 6
        self.ki_pitch = 0
        self.kd_pitch = 14.6

        # PID constants for Yaw
        self.kp_yaw = 0.3
        self.ki_yaw = 0.0
        self.kd_yaw = 0.6

        # PID constants for Throttle
        self.kp_throt = 50
        self.ki_throt = 0
        self.kd_throt = 3

        # Correction values after PID is computed
        self.correct_roll = 0.0
        self.correct_pitch = 0.0
        self.correct_yaw = 0.0
        self.correct_throt = 0.0

        # Loop time for PID computation. You are free to experiment with this
        self.last_time = 0.0
        self.loop_time = 0.010

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

    def arm(self):
        self.cmd.rcAUX4 = 1500
        self.cmd.rcThrottle = 1000


        ############################
        # self.cmd.rcRoll = 1500
        # self.cmd.rcYaw = 1500
        # self.cmd.rcPitch = 1500
        # self.cmd.rcThrottle = 1000
        # self.cmd.rcAUX4 = 1500
        ############################

        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    def disarm(self):
        self.cmd.rcAUX4 = 1100

        #############################
        # self.cmd.rcThrottle = 1300
        # self.cmd.rcAUX4 = 1200
        #############################


        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(.1)

    def position_hold(self):

        rospy.sleep(2)

        # print
        # "disarm"
        print "disarm"
        self.disarm()
        rospy.sleep(.2)
        # print
        # "arm"
        print "arm"
        self.arm()
        rospy.sleep(.1)


        while True:
            self.calc_pid()

            # Check your X and Y axis. You MAY have to change the + and the -.
            # We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
            pitch_value = int(1500 - self.correct_pitch)
            # self.cmd.rcPitch = self.limit(pitch_value, 1600, 1400)
            self.cmd.rcPitch = self.limit(pitch_value, 1575, 1425)

            roll_value = int(1500 + self.correct_roll)
            self.cmd.rcRoll = self.limit(roll_value, 1575, 1425)

            throt_value = int(1500 - self.correct_throt)
            # self.cmd.rcThrottle = self.limit(throt_value, 1650, 1350)
            self.cmd.rcThrottle = self.limit(throt_value, 1800, 1200)

            yaw_value = int(1500 + self.correct_yaw)
            self.cmd.rcYaw = self.limit(yaw_value, 1800, 1200)

            self.pluto_cmd.publish(self.cmd)

    def calc_pid(self):
        self.seconds = time.time()
        current_time = self.seconds - self.last_time
        if (current_time >= self.loop_time):
            self.pid_throt()
            # self.pid_yaw()
            self.pid_pitch()
            self.pid_roll()
            
            self.last_time = self.seconds

    # def pid_roll(self):
    #     error = self.wp_y - self.drone_y
    #     self.errorsumY += error
    #     dErr = error - self.prevErrorY
    #     PID = self.kp_roll * error + self.kd_roll * dErr / self.loop_time + self.ki_roll * self.errorsumY * self.loop_time
    #     self.prevErrorY = error
    #     self.correct_roll = PID


    # def pid_pitch(self):
    #     # Compute Pitch PID here
    #     error = self.wp_x - self.drone_x
    #     self.errorsumX += error
    #     dErr = error - self.prevErrorX
    #     PID = self.kp_pitch * error + self.kd_pitch * dErr / self.loop_time + self.ki_pitch * self.errorsumX * self.loop_time
    #     self.correct_pitch = PID

   ##################################################################################
   #Function Name:pid_roll                                                          #
   #Logic: Basic pid formula is being applied in order to calculate the correct_roll#
   #Example: self.pid_roll()                                                        #
   ##################################################################################
    def pid_roll(self):
        error = self.wp_x - self.drone_x
        self.errorsumX += error
        # self.errorsumX += (error/20)
        dErr = error - self.prevErrorX
        PID = self.kp_roll * error + self.kd_roll * dErr / self.loop_time + self.ki_roll * self.errorsumX * self.loop_time
        self.prevErrorX = error
        self.correct_roll = PID

    ####################################################################################
    # Function Name:pid_pitch                                                          #
    # Logic: Basic pid formula is being applied in order to calculate the correct_pitch#
    # Example: self.pid_pitch()                                                        #
    ####################################################################################

    def pid_pitch(self):
        # Compute Pitch PID here
        error = self.wp_y - self.drone_y
        self.errorsumY += error
        # self.errorsumY += (error/20)
        dErr = error - self.prevErrorY
        PID = self.kp_pitch * error + self.kd_pitch * dErr / self.loop_time + self.ki_pitch * self.errorsumY * self.loop_time
        self.prevErrorY = error
        self.correct_pitch = PID

    ####################################################################################
    # Function Name:pid_throt                                                          #
    # Logic: Basic pid formula is being applied in order to calculate the correct_throt#
    # Example: self.pid_throt()                                                        #
    ####################################################################################
    def pid_throt(self):
        # Compute Throttle PID here
        error = self.wp_z - self.drone_z
        self.errorsumZ += error
        # self.errorsumZ += (error/100)
        dErr = error - self.prevErrorZ
        PID = self.kp_throt * error + self.kd_throt * dErr / self.loop_time + self.ki_throt * self.errorsumZ * self.loop_time
        self.prevErrorZ = error
        self.correct_throt = PID

    ##################################################################################
    # Function Name:pid_yaw                                                          #
    # Logic: Basic pid formula is being applied in order to calculate the correct_yaw#
    # Example: self.pid_yaw()                                                        #
    ##################################################################################
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

    def our_disarm(self,inpv):
        if (inpv==5):
            self.disarm()
            print "drone disarmed"



if __name__ == '__main__':
    while not rospy.is_shutdown():
        temp = DroneFly()
        temp.position_hold()
        rospy.spin()
