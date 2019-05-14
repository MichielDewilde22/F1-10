#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

desired_trajectory = 0.5
speed = 20
vel = 30
delay_dist = vel/100
safety_dist = 0.2
car_width = 0.3
safety_angle_rad = math.atan((car_width/2)/safety_dist)
safety_angle_deg = math.ceil(math.degrees(safety_angle_rad))

pub = rospy.Publisher('error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
	if theta > 0 and theta <=180:
		index = theta*4+180
	else:
		index = -1
		
# Return the lidar scan value at that index
# Do some error checking for NaN and ubsurd values
	if index == -1:
		distance = -1
	else:
		distance = data.ranges[index]
		if distance < 0 or distance > 30:
			distance = -1
	
	return distance

def callback(data):
	theta = 50;
	start_vel = vel
	a = getRange(data,theta)
	b = getRange(data,0)
	#f = getRange(data,90)
	swing = math.radians(theta)
	
	## Your code goes here
	#Stop car if something in front of it.
	for x in range(90-safety_angle_deg, 90+safety_angle_deg, 1):
		if (getRange(data,x)<safety_dist/math.cos(math.radians(x))):
			vel = 0
			print("STOP: ",getRange(data,x)
	if (vel!=0 and start_vel == 0):
		vel = speed
	
	alpha=math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	AB=b*math.cos(alpha)
	CD=AB+delay_dist*math.sin(alpha)
	error = desired_trajectory - CD
	#if f<2:
	#	vel = 0
	print("Alfa: ", alfa, "Error: ", error)
	## END

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
