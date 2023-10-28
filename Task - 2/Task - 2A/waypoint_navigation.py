#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class swift():
	
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		self.drone_position = [0.0,0.0,0.0]	

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

		self.Kp = [50, 55, 102]
		self.Ki = [1.3, 0, 6.6]
		self.Kd = [686.25, 562.5, 3074.4]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------
		# Previous error for each axis
		self.prev_error = [0.0, 0.0, 0.0]
		self.toleranc = [0.2, 0.2, 0.2]

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
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

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


	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
	
		#---------------------------------------------------------------------------------------------------------------


	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 1 
		self.Ki[2] = alt.Ki * 0.05
		self.Kd[2] = alt.Kd * 0.8


	#-------------------------------------------------------------------------------------------------------------------
	def pitch_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.5
		self.Ki[1] = alt.Ki * 0.025
		self.Kd[1] = alt.Kd * 0.5


	def roll_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 1
		self.Ki[0] = alt.Ki * 0.05
		self.Kd[0] = alt.Kd * 0.75


	#----------------------------------------------------------------------------------------------------------------------

	def pid(self, setpoint: list[int]):

		error = [self.drone_position[i] - setpoint[i] for i in range(3)]

		iterm = [0,0,0]

		#Calculating For throttle
		tP = self.Kp[2] * error[2]
		tI = self.Ki[2] * iterm[2]
		tD = self.Kd[2] * (error[2] - self.prev_error[2])

		throttle = 1590 + int(tP + tI + tD)
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
		self.pub_pitch_error.publish(error[1])
		self.pub_roll_error.publish(error[0])		

	#------------------------------------------------------------------------------------------------------------------------
		self.pub_drone.publish(self.cmd)
	#------------------------------------------------------------------------------------------------------------------------
		setpoint_reached = all(abs(err) < tolerance for err, tolerance in zip(error, self.tolerance))

		return setpoint_reached
    
		

if __name__ == '__main__':

	setpoint = [[0, 0, 23], [2, 0, 23], [2, 2, 23], [2, 2, 25], [-5, 2, 25], [-5, -3, 25], [-5, -3, 21], [7, -3, 21], [7, 0, 21], [0, 0, 19]]
	waypoint = 0
	swift_drone = swift()
	rate = rospy.Rate(1/swift_drone.sampletime)
	while not rospy.is_shutdown() and waypoint < len(setpoint):
		reached = swift_drone.pid(setpoint[waypoint])
		if reached: 
			waypoint+=1
		rate.sleep()