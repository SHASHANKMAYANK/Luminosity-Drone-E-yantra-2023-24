#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

import time as t
from swift_msgs.msg import *
from luminosity_drone.msg import Biolocation
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge ,CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library
import numpy as np

class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [-6, -7,20]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly5

		self.wp_setpoints =[ [-6,-7,20],
                                         [ 0,-7,20],
                                         [ 6,-7,20],
                                          [6,0,20],
                                   	 	[0,0,20],
                                    	[-6,0,20],
                                          [-6,7,20],
                                          [0,7, 20],
                                          [6,7, 20]	]

		self.prev_length=len(self.wp_setpoints)
		self.alien_location=[]
		self.alien_type = []
		self.centroid_alien=[]
		self.i = 0
		self.a = 0
		self.h = 0
		self.prev_h=0
		self.z=0
		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		self.cdd = Biolocation()
		self.cdd.organism_type = ''
		self.cdd.whycon_x = 0.0
		self.cdd.whycon_y = 0.0
		self.cdd.whycon_z = 0.0

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [96.96, 27.54, 30.26]
		self.Ki = [1.048, 1.048, 0.0]
		self.Kd = [622.2, 570, 550.2]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------
		

		self.roll_prev_error = 0                                                                            # for x_axis
		self.pitch_prev_error = 0                                                                           # for y_axis
		self.throttle_prev_error = 0                                                                        # for z_axis 
		self.led = 0
		self.j = 0
		self.cax = 0
		self.cay = 0
		self.dx=0
		self.dy=0
		self.roll_error = 0                                                                                 # for x_axis
		self.pitch_error = 0                                                                                # for y_axis
		self.throttle_error = 0                                                                             # for z_axis 
		
		self.roll_sum_error = 0                                                                             # for x_axis
		self.pitch_sum_error = 0                                                                            # for y_axis
		self.throttle_sum_error = 0                                                                         # for z_axis 
		
		self.min_roll = 1200 
		self.max_roll = 1800   
		self.min_pitch = 1200 
		self.max_pitch = 1800
		self.min_throttle = 1325
		self.max_throttle = 1745

		self.max_error_roll = 0.1
		self.min_error_roll = -0.1
		self.max_error_pitch = 0.1
		self.min_error_pitch = -0.1
		self.max_error_throttle = 0.01
		self.min_error_throttle = -0.01

		self.sample_time = 0.050



		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.throttle_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.alien_and_location_pub = rospy.Publisher('/astrobiolocation',Biolocation, queue_size=1)



	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		rospy.Subscriber('/drone_display/image_view/output', Image, self.callback)


		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06# This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06 
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06 
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3

	def callback(self,data):


		# Used to convert between ROS and OpenCV images
		self.br = CvBridge()

		# Output debugging information to the terminal

		# Convert ROS Image message to OpenCV image
		self.current_frame = self.br.imgmsg_to_cv2(data)

		# Display image

		self.gray = cv.cvtColor(self.current_frame, cv.COLOR_BGR2GRAY)
		self.bilateral = cv.bilateralFilter(self.gray, 1, 15, 15)
		ret, self.thresh = cv.threshold(self.bilateral, 200, 255, cv.THRESH_BINARY)
		self.dilated = cv.dilate(self.thresh, (3, 3), iterations=3)
		self.eroded = cv.erode(self.dilated, (5, 5), iterations=3)
		self.blank = np.zeros(self.current_frame.shape[:2], dtype='uint8')
		self.b, self.g, self.r = cv.split(self.current_frame)
		contours, hierarchies = cv.findContours(self.eroded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		contours1, hierarchies1 = cv.findContours(self.thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		self.areas = []
		self.centroid = []
		self.led = 0
		cv.drawContours(self.blank, contours1, -1, (255, 255, 255), 8)
		self.bitwise_add = cv.bitwise_or(self.r, self.blank)
		self.merged = cv.merge([self.b, self.g, self.bitwise_add])
		cv.imshow('merged', self.merged)

		for i in range(0, len(contours), 1):
			cnt = contours[i]
			M = cv.moments(cnt)
			area = cv.contourArea(cnt)
			if (M['m00']>0 and area>0):
				cx = M['m10'] / M['m00']
				cy = M['m01'] / M['m00']
				self.areas.append(area)
				self.centroid.append((cx, cy))
				self.led += 1

		self.j = 0
		self.cax = 0
		self.cay = 0
		self.centroid_alien = []

		for self.i in range(1, len(self.centroid), 1):
			self.diffx = self.centroid[self.i][0] - self.centroid[self.i - 1][0]
			self.diffy = self.centroid[self.i][1] - self.centroid[self.i - 1][1]
			if (self.i == len(contours) - 1):
				self.j = self.j + 1
				for self.k in range(0, self.j + 1, 1):
					self.cax += self.centroid[self.i - self.k][0]
					self.cay += self.centroid[self.i - self.k][1]
				self.centroid_alien.append([(self.cax) / (self.j + 1), (self.cay) / (self.j + 1)])
				self.h = self.j
			elif (-100 <= self.diffx <= 100 or -100 <= self.diffy <= 100):
				self.j = self.j + 1
			elif (self.diffx > 100 or self.diffx < -100 or self.diffy > 100 or self.diffy < -100):
				for self.z in range(0, self.j + 1, 1):
					self.cax += self.centroid[(self.i - 1) - self.z][0]
					self.cay += self.centroid[(self.i - 1) - self.z][1]
				self.centroid_alien.append([(self.cax) / (self.j + 1), (self.cay) / (self.j + 1)])
				self.h = self.j
				self.cay = 0
				self.cax = 0
				self.j = 0
		cv.waitKey(1)

	def perfect_landing(self):
		if (self.setpoint[2]>34):

				self.cmd.rcRoll = 1500
				self.cmd.rcYaw = 1500
				self.cmd.rcPitch = 1500
				self.cmd.rcThrottle = 1570
				self.cmd.rcAUX4 = 1500
				self.command_pub.publish(self.cmd)  # Publishing /drone_command
				rospy.sleep(5)
				rospy.loginfo(self.alien_type[0])
				rospy.loginfo(self.drone_position[0])
				rospy.loginfo(self.drone_position[1])
				rospy.loginfo(self.drone_position[2])

				self.cdd.organism_type = self.alien_type[0]
				self.cdd.whycon_x = self.drone_position[0]
				self.cdd.whycon_y = self.drone_position[1]
				self.cdd.whycon_z = self.drone_position[2]

				self.alien_and_location_pub.publish(self.cdd)

				if (len(self.alien_type) > 1):
					for self.z in range(1,len(self.alien_location),1):
						self.setpoint[0] = self.alien_location[self.z][0]
						self.setpoint[1] = self.alien_location[self.z][1]
						self.setpoint[2] = self.alien_location[self.z][2]
				else:
					self.cdd.organism_type = self.alien_type[2]
					self.cdd.whycon_x = self.drone_position[0]
					self.cdd.whycon_y = self.drone_position[1]
					self.cdd.whycon_z = self.drone_position[2]

					self.alien_and_location_pub.publish(self.cdd)


					self.setpoint[0] = 11
					self.setpoint[1] = 11
					self.setpoint[2] = 37


		elif (((self.setpoint[0] - 0.19) <= self.drone_position[0] <= (self.setpoint[0] + 0.19)) and (
				(self.setpoint[1] - 0.19) <= self.drone_position[1] <= (self.setpoint[1] + 0.19)) and (
				(self.setpoint[2] - 0.19) <= self.drone_position[2] <= (self.setpoint[2] + 0.19))):
			for self.o in range(0, len(self.centroid_alien), 1):
				self.middlex = self.current_frame.shape[0] / 2
				self.middley = self.current_frame.shape[1] / 2
				self.dx = (self.middlex - self.centroid_alien[self.o][0])
				self.dy = (self.middley - self.centroid_alien[self.o][1])
			rospy.loginfo(self.dx)
			rospy.loginfo(self.dy)
			self.setpoint[0] = self.setpoint[0] - (self.dx / 200)
			self.setpoint[1] = self.setpoint[1] - (self.dy / 200)
			self.setpoint[2] = self.setpoint[2] + 3
			





	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum


		self.roll_error = self.drone_position[0] - self.setpoint[0]                                                       #error[0] corresponds to error in x...
		self.pitch_error = self.drone_position[1] - self.setpoint[1]                                                      #error[1] corresponds to error in y...
		self.throttle_error= self.drone_position[2] - self.setpoint[2]                                                    #error[2] corresponds to error in z...
	
		 
		
		
		self.cmd.rcRoll = int(1500 - (self.Kp[0] * self.roll_error + self.Kd[0] * (self.roll_error - self.roll_prev_error) + self.Ki[0] * self.roll_sum_error))
		self.cmd.rcPitch = int(1500 + (self.Kp[1] * self.pitch_error + self.Kd[1] * (self.pitch_error - self.pitch_prev_error) + self.Ki[1] * self.pitch_sum_error))
		self.cmd.rcThrottle = int (1583+ (self.throttle_error * self.Kp[2] +(self.throttle_error - self.throttle_prev_error) * self.Kd[2] + self.throttle_sum_error * self.Ki[2]))


		if self.cmd.rcRoll > self.max_roll:
			self.cmd.rcRoll = self.max_roll
		if self.cmd.rcRoll < self.min_roll:
			self.cmd.rcRoll = self.min_roll
	
		if self.cmd.rcPitch > self.max_pitch:
			self.cmd.rcPitch = self.max_pitch
		if self.cmd.rcPitch < self.min_pitch:
			self.cmd.rcPitch = self.min_pitch
					
		if self.cmd.rcThrottle > self.max_throttle:
			self.cmd.rcThrottle = self.max_throttle
		if self.cmd.rcThrottle < self.min_throttle:
			self.cmd.rcThrottle = self.min_throttle
		
					#anti-windup	
		
		if self.roll_sum_error > self.max_error_roll:
			self.roll_sum_error = self.max_error_roll
		if self.roll_sum_error < self.min_error_roll:
			self.roll_sum_error = self.min_error_roll
	
		if self.pitch_sum_error > self.max_error_pitch:
			self.pitch_sum_error = self.max_error_pitch
		if self.pitch_sum_error < self.min_error_pitch:
			self.pitch_sum_error = self.min_error_pitch
					
		if self.throttle_sum_error > self.max_error_throttle:
			self.throttle_sum_error = self.max_error_throttle
		if self.throttle_sum_error < self.min_error_throttle:
			self.throttle_sum_error = self.min_error_throttle
			
			
		self.roll_prev_error = self.roll_error
		self.pitch_prev_error = self.pitch_error 
		self.throttle_prev_error = self.throttle_error
		
		self.roll_sum_error = self.roll_sum_error + self.roll_error                                                       #s_error[0] (sum of error) corresponds to error in x...  
		self.pitch_sum_error = self.pitch_sum_error + self.pitch_error                                                    #s_error[1] (sum of error) corresponds to error in y...  
		self.throttle_sum_error = self.throttle_sum_error + self.throttle_error                                           #s_error[2] (sum of error) corresponds to error in z... 	


		if (self.setpoint[0] == 11 and self.setpoint[1] == 11 and self.setpoint[2] == 37
and (
				(self.setpoint[0] -0.2) <= self.drone_position[0] <= (self.setpoint[0] + 0.2)) and (
				(self.setpoint[1] - 0.2) <= self.drone_position[1] <= (self.setpoint[1] + 0.2)) and (
				(self.setpoint[2] - 0.2) <= self.drone_position[2] <= (self.setpoint[2] + 0.2))):
				
					

						self.cmd.rcRoll = 1500
						self.cmd.rcYaw = 1500
						self.cmd.rcPitch = 1500
						self.cmd.rcThrottle = 1500
						self.cmd.rcAUX4 = 1500
						self.command_pub.publish(self.cmd)  # Publishing /drone_command
						rospy.sleep(5)
						rospy.loginfo(self.drone_position[0])
						rospy.loginfo(self.drone_position[1])
						rospy.loginfo(self.drone_position[2])

		elif (self.z>0 and (
				(self.setpoint[0] + 0.05) <= self.drone_position[0] <= (self.setpoint[0] + 0.20)) and (
				(self.setpoint[1] - 0.05) <= self.drone_position[1] <= (self.setpoint[1] + 0.05)) and (
				(self.setpoint[2] - 0.2) <= self.drone_position[2] <= (self.setpoint[2] + 0.0))):
				self.perfect_landing()

		elif (self.setpoint[2]>20 and ((self.setpoint[0] - 0.19) <= self.drone_position[0] <= (self.setpoint[0] + 0.19)) and (
				(self.setpoint[1] - 0.19) <= self.drone_position[1] <= (self.setpoint[1] + 0.19)) and (
				(self.setpoint[2] - 0.19) <= self.drone_position[2] <= (self.setpoint[2] + 0.19))):
			self.perfect_landing()

		elif (len(self.wp_setpoints)>self.prev_length and  ((self.setpoint[0] - 0.19) <= self.drone_position[0] <= (self.setpoint[0] + 0.19)) and ((self.setpoint[1] - 0.19) <= self.drone_position[1] <= (self.setpoint[1] + 0.19)) and ((self.setpoint[2] - 0.19) <= self.drone_position[2] <= (self.setpoint[2] + 0.19))):
					rospy.loginfo(self.setpoint[0])
					rospy.loginfo(self.setpoint[1])
					rospy.loginfo(self.setpoint[2])
					if (self.setpoint[0]==0 and self.setpoint[1]==0 and self.setpoint[2]==20 ):
						rospy.loginfo(self.alien_location)
						self.setpoint[0] = self.alien_location[0][0]
						self.setpoint[1] = self.alien_location[0][1]
						self.setpoint[2] = self.alien_location[0][2]
						if (((self.setpoint[0] - 0.19) <= self.drone_position[0] <= (self.setpoint[0] + 0.19)) and (
								(self.setpoint[1] - 0.19) <= self.drone_position[1] <= (self.setpoint[1] + 0.19)) and (
								(self.setpoint[2] - 0.19) <= self.drone_position[2] <= (self.setpoint[2] + 0.19))):

							for self.o in range(0, len(self.centroid_alien), 1):
								self.middlex = self.current_frame.shape[0] / 2
								self.middley = self.current_frame.shape[1] / 2
								self.dx = (self.middlex - self.centroid_alien[self.o][0])
								self.dy = (self.middley - self.centroid_alien[self.o][1])
							rospy.loginfo(self.dx)
							rospy.loginfo(self.dy)
							self.setpoint[0] = self.setpoint[0] - (self.dx / 200)
							self.setpoint[1] = self.setpoint[1] - (self.dy / 200)
							self.setpoint[2] = self.setpoint[2] + 3
					else:
							rospy.loginfo(self.alien_location)
							for self.a in range(0, len(self.alien_location), 1):
								self.setpoint[0] = self.alien_location[self.a][0]
								self.setpoint[1] = self.alien_location[self.a][1]
								self.setpoint[2] = self.alien_location[self.a][2]
								if (((self.setpoint[0] - 0.19) <= self.drone_position[0] <= (self.setpoint[0] + 0.19)) and (
										(self.setpoint[1] - 0.19) <= self.drone_position[1] <= (self.setpoint[1] + 0.19)) and (
										(self.setpoint[2] - 0.19) <= self.drone_position[2] <= (self.setpoint[2] + 0.19))):

									for self.o in range(0, len(self.centroid_alien), 1):
										self.middlex = self.current_frame.shape[0] / 2
										self.middley = self.current_frame.shape[1] / 2
										self.dx = (self.middlex - self.centroid_alien[self.o][0])
										self.dy = (self.middley - self.centroid_alien[self.o][1])
									rospy.loginfo(self.dx)
									rospy.loginfo(self.dy)
									self.setpoint[0] = self.setpoint[0] - (self.dx / 200)
									self.setpoint[1] = self.setpoint[1] - (self.dy / 200)
									self.setpoint[2] = self.setpoint[2] + 3



		else:
				for self.i in range(0, len(self.wp_setpoints), 1):
					if ((self.setpoint[0] == self.wp_setpoints[self.i][0] and self.setpoint[1] == self.wp_setpoints[self.i][1] and
						 self.setpoint[2] == self.wp_setpoints[self.i][2]) and (
							(self.setpoint[0] - 0.19) <= self.drone_position[0] <= (self.setpoint[0] + 0.19)) and (
							(self.setpoint[1] - 0.19) <= self.drone_position[1] <= (self.setpoint[1] + 0.19)) and (
							(self.setpoint[2] - 0.19) <= self.drone_position[2] <= (self.setpoint[2] + 0.19)) and len(
						self.centroid_alien) == 0):
						if (self.i == len(self.wp_setpoints) - 1):
							self.wp_setpoints=self.wp_setpoints+self.alien_location
							print(self.i)

							self.setpoint[0] = 0
							self.setpoint[1] = 0
							self.setpoint[2] = 20



						else:
							self.setpoint[0] = self.wp_setpoints[self.i + 1][0]
							self.setpoint[1] = self.wp_setpoints[self.i + 1][1]
							self.setpoint[2] = self.wp_setpoints[self.i + 1][2]
							print(self.i)

					elif ((self.setpoint[0] == self.wp_setpoints[self.i][0] and self.setpoint[1] == self.wp_setpoints[self.i][1] and
						   self.setpoint[2] == self.wp_setpoints[self.i][2]) and (
								  (self.setpoint[0] - 0.19) <= self.drone_position[0] <= (self.setpoint[0] + 0.19)) and (
								  (self.setpoint[1] - 0.19) <= self.drone_position[1] <= (self.setpoint[1] + 0.19)) and (
								  (self.setpoint[2] - 0.19) <= self.drone_position[2] <= (self.setpoint[2] + 0.19)) and len(
						self.centroid_alien) > 0):

						for self.o in range(0, len(self.centroid_alien), 1):
							self.middlex = self.current_frame.shape[0] / 2
							self.middley = self.current_frame.shape[1] / 2
							self.dx = (self.middlex - self.centroid_alien[self.o][0])
							self.dy = (self.middley - self.centroid_alien[self.o][1])
						if (-500 <= self.dx <= 500 and -500 <= self.dy <= 500):
							rospy.loginfo(self.dx)
							rospy.loginfo(self.dy)
							if (self.prev_h==0 and self.h > 0):
								self.alien_location.append([self.setpoint[0], self.setpoint[1], self.setpoint[2]])
								if (self.h == 2):
									self.alien_type.append('alien_b')
								elif (self.h == 1):
									self.alien_type.append('alien_a')
								elif (self.h == 3):
									self.alien_type.append('alien_c')
							elif(self.h == self.prev_h):
								pass
							else:
								self.alien_location.append([self.setpoint[0], self.setpoint[1], self.setpoint[2]])
								if (self.h == 2):
									self.alien_type.append('alien_b')
								elif (self.h == 1):
									self.alien_type.append('alien_a')
								elif (self.h == 3):
									self.alien_type.append('alien_c')

							self.prev_h=self.h
							rospy.loginfo(self.alien_location)
							rospy.loginfo(self.alien_type)
							if (self.i == len(self.wp_setpoints) - 1):

								self.wp_setpoints = self.wp_setpoints + self.alien_location
								print(self.i)


							else:
								self.setpoint[0] = self.wp_setpoints[self.i + 1][0]
								self.setpoint[1] = self.wp_setpoints[self.i + 1][1]
								self.setpoint[2] = self.wp_setpoints[self.i + 1][2]
							print(self.i)

						else:
							if (self.i == len(self.wp_setpoints) - 1):

								self.wp_setpoints = self.wp_setpoints + self.alien_location
								print(self.i)


							else:
								self.setpoint[0] = self.wp_setpoints[self.i + 1][0]
								self.setpoint[1] = self.wp_setpoints[self.i + 1][1]
								self.setpoint[2] = self.wp_setpoints[self.i + 1][2]
							print(self.i)

		# -----------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		self.roll_error_pub.publish(self.roll_error)
		self.pitch_error_pub.publish(self.pitch_error)
		self.throttle_error_pub.publish(self.throttle_error)

if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(20) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()
		r.sleep()

