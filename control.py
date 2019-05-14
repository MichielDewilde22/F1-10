#!/usr/bin/env python

import rospy
from race.msg import drive_param
from race.msg import pid_input

kp = 14.0
kd = 0.09
servo_offset = 0 #18.5	# zero correction offset in case servo is misaligned. 
prev_error = 0.0 
vel_input = 25.0
scale_factor = 10
max_angle = 45

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd

	## Your code goes here
	# 1. Scale the error
	error = scale_factor*data.pid_error
	# 2. Apply the PID equation on error
	V0 = Kp*error + Kd*prev_error-error
	# 3. Make sure the error is within bounds
	if V0 < max_angle and V0 > -max_angle:
                angle = V0
	elif V0 > max_angle:
                angle = max_angle
	elif V0 < -max_angle:
                angle = -max_angle
	else:
                angle = 0
	angle+=servo_offset
	print("Angle: ", angle)
	## END

	msg = drive_param();
	if data.pid_vel == 0:
		msg.velocity = 0;
	else:
		msg.velocity = vel_input	
	msg.angle = angle
	pub.publish(msg)

if __name__ == '__main__':
	#global kp
	#global kd
	#global vel_input
	print("Listening to error for PID")
	#kp = input("Enter Kp Value: ")
	#kd = input("Enter Kd Value: ")
	#vel_input = input("Enter Velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
