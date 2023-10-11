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

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2,2,20] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

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

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [26, 10, 102]
		self.Ki = [1.3, 0, 6.6]
		self.Kd = [54.75, 5, 3074.4]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------
		# Previous error for each axis
		self.prev_error = [0.0, 0.0, 0.0]

		# Maximum and minimum RC values for roll, pitch, and throttle
		self.max_values = [1600, 1600, 1800]
		self.min_values = [1400, 1400, 1500]

		# Sample time for PID control (in seconds)
		self.sampletime = 0.033

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.pub_drone = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)

		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.pub_alt_error = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pub_pitch_error = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.pub_roll_error = rospy.Publisher('/roll_error', Float64, queue_size=1)

	#-----------------------------------------------------------------------------------------------------------

		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll

		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)

		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pub_drone.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.pub_drone.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
	
		#---------------------------------------------------------------------------------------------------------------


	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 1 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.05
		self.Kd[2] = alt.Kd * 0.8


	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.5
		self.Ki[1] = alt.Ki * 0.025
		self.Kd[1] = alt.Kd * 0.5


	def roll_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 1
		self.Ki[0] = alt.Ki * 0.05
		self.Kd[0] = alt.Kd * 0.75


	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):

	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

		error = [self.drone_position[i] - self.setpoint[i] for i in range(3)]

		iterm = [0,0,0]

		#Calculating For throttle
		tP = self.Kp[2] * error[2]
		tI = self.Ki[2] * iterm[2]
		tD = self.Kd[2] * (error[2] - self.prev_error[2])

		throttle = 1500 + int(tP + tI + tD)
		if throttle > self.max_values[2]:
			throttle = self.max_values[2]
		elif throttle < self.min_values[2]:
			throttle = self.min_values[2]
		
		self.prev_error[2] = error[2]
		iterm[2] += error[2]

		#Calculating For roll
		rP = self.Kp[0] * error[0]
		rI = self.Ki[0] * iterm[0]
		rD = self.Kd[0] * (error[0] - self.prev_error[0])

		roll = 1500 - int(rP + rI + rD)
		if roll > self.max_values[0]:
			roll = self.max_values[0]
		elif roll < self.min_values[0]:
			roll = self.min_values[0]
				
		self.prev_error[0] = error[0]
		iterm[0] += error[0]

		#Calculating For pitch
		pP = self.Kp[1] * error[1]
		pI = self.Ki[1] * iterm[1]
		pD = self.Kd[1] * (error[1] - self.prev_error[1])

		pitch = 1500 + int(pP + pI + pD)
		if pitch > self.max_values[1]:
			pitch = self.max_values[1]
		elif pitch < self.min_values[1]:
			pitch = self.min_values[1]
		
		self.prev_error[1] = error[1]
		iterm[1] += error[1]

		# Publish PID output as drone commands (SwiftMsg)
		self.cmd.rcRoll = int(roll)  # Roll control
		self.cmd.rcPitch = int(pitch)  # Pitch control
		self.cmd.rcThrottle = int(throttle)  # Throttle control
		self.pub_drone.publish(self.cmd)	

		# Publish error values
		self.pub_alt_error.publish(error[2])
		self.pub_pitch_error.publish(error[0])
		self.pub_roll_error.publish(error[1])		

	#------------------------------------------------------------------------------------------------------------------------
		self.pub_drone.publish(self.cmd)
		

if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(1/swift_drone.sampletime) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()
		r.sleep()