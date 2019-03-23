#!/usr/bin/env python
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import *
import rospy
import time
import datetime
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
        #print ("Start")
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
        rospy.Subscriber('/red',Int32, self.CountRed)
        rospy.Subscriber('/led',Int32, self.CountLed)
        rospy.Subscriber('/blue',Int32, self.CountBlue)
        rospy.Subscriber('/green',Int32, self.CountGreen)
        
        self.cmd = PlutoMsg()

        self.thresh = 2
        self.threshZ = 1.5

        w,h = 3,2
         ##Plant_location array that stores the whycon coordinates of all the plants
        self.Plant_location = [[0.0 for x in range(w)] for y in range(h)]                     

        self.Plant_location[0] = [-5.4, 2.8, 24.3]
        self.Plant_location[1] = [-2.5, -5.5, 20.7]
        
        w,h = 3,8
        self.points = [[0.0 for x in range(w)] for y in range(h)]
        self.points[0] = [-1.5,5.78,23]

        # Red
        #self.points[1] = [-5.45, 0.34, 22.8]
        self.points[1] = self.Plant_location[0]

        self.points[2]  = [(self.Plant_location[0][0]+self.Plant_location[1][0])/2, (self.Plant_location[0][1]+self.Plant_location[1][1])/2, min(self.Plant_location[1][2],self.Plant_location[0][2])-0.5]

        # Green 
        #self.points[2] = [-0.15 , -4.3 , 21.8]
        self.points[3] = self.Plant_location[1]

       

        self.points[5] = [0.02,5.78,21.31]
        self.points[4] = [(self.points[5][0]+self.Plant_location[1][0])/2, (self.points[5][1]+self.Plant_location[1][1])/2, min(self.Plant_location[1][2],self.points[5][2])-0.5]
        self.points[6] = [-0.98, 6.04, 24.63]
        #self.points[7] = [0.04 , 5.9, 27.3]
        self.points[7] = [-0.17 , 5.75, 28]
        # Position to hold.
        self.wp_x = self.points[0][0]
        self.wp_y = self.points[0][1]
        self.wp_z = self.points[0][2]
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
        self.kp_roll = 10
        self.ki_roll = 0
        # self.kd_roll = 4.2
        self.kd_roll = 5
        # self.kd_roll = 1.0

        # PID constants for Pitch
        self.kp_pitch = 10
        self.ki_pitch = 0
        self.kd_pitch = 14.6

        # PID constants for Yaw
        self.kp_yaw = 0.3
        self.ki_yaw = 0.0
        self.kd_yaw = 0.6

        # PID constants for Throttle
        self.kp_throt = 60
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
        self.pluto_cmd.publish(self.cmd)
        rospy.sleep(1)

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
        self.count_color = 0
        self.count_red =0
        self.count_green = 0
        self.count_blue = 0
        
        self.counter = 0
        self.c = 0
        self.timer = 0
        self.time_count = 0
        self.time_count1 = 0
        self.time_count2 = 0
        self.time_count3 = 0
        self.time_count4 = 0
        self.point_skipped = 0
        self.c1 = 0
        while True:
            self.calc_pid()
            #print self.count_color
            if (time.time()-self.timer > 45) and self.c ==1:
                print ("Stop")
                self.disarm()
                break
            if self.counter == 1 or self.counter == 3 :
                if (time.time()-self.time_count > 25) and self.c1 == 0:
                    print ("25s done")
                    self.time_count1 = time.time()
                    self.wp_x = self.points[self.counter][0] + 1
                    self.wp_y = self.points[self.counter][1] + 1
                    self.wp_z = self.points[self.counter][2]
                    self.c1 = 1
                elif (time.time() - self.time_count1 > 5) and self.c1 == 1:
                    print ("5s done 1")
                    self.time_count2 = time.time()
                    self.wp_x = self.points[self.counter][0] - 1
                    self.wp_y = self.points[self.counter][1] - 1
                    self.wp_z = self.points[self.counter][2]
                    self.c1 = 2
                elif (time.time() - self.time_count2 > 5) and self.c1 == 2:
                    print ("5s done 2")
                    self.time_count3 = time.time()
                    self.wp_x = self.points[self.counter][0] + 1
                    self.wp_y = self.points[self.counter][1] - 1
                    self.wp_z = self.points[self.counter][2]
                  
                    self.c1 = 3

                elif (time.time() - self.time_count3>5) and self.c1 == 3:
                    print("5s done 3")
                    self.time_count4 = time.time()
                    self.wp_x = self.points[self.counter][0] - 1
                    self.wp_y = self.points[self.counter][1] + 1
                    self.wp_z = self.points[self.counter][2]
                    self.c1 = 4
                # elif self.c1 == 4 :
                #     self.counter = self.counter+1
                #     self.wp_x = self.points[self.counter][0]
                #     self.wp_y = self.points[self.counter][1]
                #     self.wp_z = self.points[self.counter][2]
                #     self.c1 = 0
                #     self.point_skipped +=1


            if self.counter == 1 and self.count_color == 1:
                self.counter = self.counter+1
                self.wp_x = self.points[self.counter][0]
                self.wp_y = self.points[self.counter][1]
                self.wp_z = self.points[self.counter][2]
                #print ("done", self.counter)

            if (self.counter == 3 ) and self.count_color == 2 - self.point_skipped :
                self.counter = self.counter+1
                self.wp_x = self.points[self.counter][0]
                self.wp_y = self.points[self.counter][1]
                self.wp_z = self.points[self.counter][2]
                #print ("done", self.counter)


            if (self.counter == 0 or self.counter == 2 or self.counter ==4 ) and abs(self.drone_x - self.wp_x) < 2 and abs(self.drone_y-self.wp_y)< 2 and abs(self.drone_z - self.wp_z) < 1:
                self.counter = self.counter+1
                
                self.wp_x = self.points[self.counter][0]
                self.wp_y = self.points[self.counter][1]
                self.wp_z = self.points[self.counter][2]
                #print ("done", self.counter)
                self.time_count = time.time()
                self.c1 = 0

            
            if self.counter == 5 and self.c==0:
                if self.count_red>0:
                    print "Pollination Done! Pollinated", self.count_red ,"Red Daylily, " , 
                if self.count_green>0:
                    print self.count_green,  "Green Carnation" , 

                if self.count_blue>0:
                    print "and ",self.count_blue ," Blue Delphinium"
                self.c+=1
                self.timer = time.time()


            ############# TODO #######################
            ######### ONE KEY DISARM #################
                

            if self.counter == 5 and abs(self.drone_x - self.wp_x) < 2 and abs(self.drone_y-self.wp_y)< 2 and abs(self.drone_z - self.wp_z) < 2:
                self.counter = self.counter + 1
                self.wp_x = self.points[self.counter][0]
                self.wp_y = self.points[self.counter][1]
                self.wp_z = self.points[self.counter][2]
                #print ("done", self.counter)
                
            if self.counter == 6 and abs(self.drone_x - self.wp_x) < 1 and abs(self.drone_y- self.wp_y)< 1 and abs(self.drone_z - self.wp_z) < 1:
                self.counter = self.counter + 1
                self.wp_x = self.points[self.counter][0]
                self.wp_y = self.points[self.counter][1]
                self.wp_z = self.points[self.counter][2]
                
                


            if self.counter == 7 and abs(self.drone_x - self.wp_x) < 0.4 and abs(self.drone_y- self.wp_y)< 0.4 and abs(self.drone_z - self.wp_z) < 0.8 and self.c==1:
                self.c += 1  
                print ("STOP")
                self.disarm()
                 

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

    def CountLed(self, data):
        self.count_color  = data.data
    
    def CountRed(self, data):
        self.count_red  = data.data
    
    def CountBlue(self, data):
        self.count_blue  = data.data
    
    def CountGreen(self, data):
        self.count_green  = data.data
    

if __name__ == '__main__':
    print ("START")
    while not rospy.is_shutdown():
        temp = DroneFly()
        temp.position_hold()
        rospy.spin()
