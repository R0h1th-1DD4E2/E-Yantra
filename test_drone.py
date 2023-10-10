#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class SwiftDroneController():
    def __init__(self):
        rospy.init_node('drone_control')

        # Initialize drone position and setpoint
        self.drone_position = [0.0, 0.0, 0.0]
        self.setpoint = [2, 2, 20]  # Example setpoint [x, y, z]

        # Initialize PID gains
        self.Kp = [1, 1, 1]
        self.Ki = [0.001, 0.001, 0.001]
        self.Kd = [0.1, 0.1, 0.1]

        # Initialize other variables for PID control
        self.prev_error = [0, 0, 0]  # Previous errors for [pitch, roll, throttle]
        self.max_values = [2000, 2000, 2000]  # Maximum values for [roll, pitch, throttle]
        self.min_values = [1000, 1000, 1000]  # Minimum values for [pitch, roll, throttle]
        self.sample_time = 0.060  # Sample time for PID control (adjust as needed)

        # Initialize variables for integral term
        self.error_sum = [0.0, 0.0, 0.0]

        # Initialize the drone command
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # ROS Publishers
        self.pub_drone = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        self.pub_alt_error = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.pub_pitch_error = rospy.Publisher('/pitch_error', Float64, queue_size=1)
        self.pub_roll_error = rospy.Publisher('/roll_error', Float64, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)

        # Arm the drone
        self.arm()
    

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.pub_drone.publish(self.cmd)
        rospy.sleep(1)
    

    def arm(self):
        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.pub_drone.publish(self.cmd)
        rospy.sleep(1)


    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        # Update the y coordinate
        self.drone_position[1] = msg.poses[0].position.y
        # Update the z coordinate
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.01
        self.Ki[2] = alt.Ki * 0.0001
        self.Kd[2] = alt.Kd * 0.1

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.01
        self.Ki[1] = pitch.Ki * 0.0001
        self.Kd[1] = pitch.Kd * 0.1

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.01
        self.Ki[0] = roll.Ki * 0.0001
        self.Kd[0] = roll.Kd * 0.1

    def pid(self):
        # Calculate errors for each axis
        error = [self.setpoint[i] - self.drone_position[i] for i in range(3)]

        # Initialize control outputs
        control_output = [0.0, 0.0, 0.0]

        # PID loop for each axis
        for axis in range(3):
            # Proportional term
            P = self.Kp[axis] * error[axis]

            # Integral term (sum of errors)
            self.error_sum[axis] += error[axis] * self.sample_time
            I = self.Ki[axis] * self.error_sum[axis]

            # Derivative term (rate of change of error)
            D = self.Kd[axis] * (error[axis] - self.prev_error[axis]) / self.sample_time

            # Total control output
            control_output[axis] = P + I + D

            # Limit control output within bounds
            if control_output[axis] > self.max_values[axis]:
                control_output[axis] = self.max_values[axis]
            elif control_output[axis] < self.min_values[axis]:
                control_output[axis] = self.min_values[axis]

            # Update previous error
            self.prev_error[axis] = error[axis]

        # Update the drone command
        self.cmd.rcPitch = 1500 + control_output[1]  # Adjust for pitch control
        self.cmd.rcRoll = 1500 + control_output[0]   # Adjust for roll control
        self.cmd.rcThrottle = 1500 + control_output[2]  # Adjust for throttle control

        # Publish the drone command
        self.pub_drone.publish(self.cmd)
        self.pub_alt_error.publish(self.cmd.rcThrottle)
        self.pub_pitch_error.publish(self.cmd.rcPitch)
        self.pub_roll_error.publish(self.cmd.rcRoll)


if __name__ == '__main__':
    swift_drone = SwiftDroneController()
    r = rospy.Rate(1 / swift_drone.sample_time)  # Set the rate based on the sample time
    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()
