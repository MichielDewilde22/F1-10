#!/usr/bin/env python

import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

desired_trajectory = 0.5
speed = 35
vel = 35
delay_dist = vel/100
safety_dist = 1
car_width = 0.4 #0.3
safety_angle_rad = math.atan((car_width/2)/safety_dist)
safety_angle_deg = math.degrees(safety_angle_rad)


pub = rospy.Publisher('error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
	if theta >= 0 and theta <=180:
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
	global vel
	theta = 50
	start_vel = vel
	vel = speed
	a = getRange(data,theta)
	b = getRange(data,0)
	#f = getRange(data,90)
	swing = math.radians(theta)
	
	## Your code goes here
	#Stop car if something in front of it.
	#for x in range(int(math.floor(90-safety_angle_deg)), int(math.ceil(90+safety_angle_deg)), int(safety_angle_deg)):
	for x in numpy.arange(math.floor(90-safety_angle_deg), math.ceil(90+safety_angle_deg),1):
		dist = getRange(data,int(round(x)))	
		if (dist*math.sin(math.radians(x))<safety_dist and dist!=-1): #math.sin(math.radians(x))):
			vel = 0
			print("STOP: ",dist,"Angle: ",x)
	if (vel!=0 and start_vel == 0):
		vel = speed
	
	alpha=round(math.atan((a*math.cos(swing)-b)/(a*math.sin(swing))),2)
	alpha_deg=round(math.degrees(alpha),2)
	AB=b*math.cos(alpha)
	CD=AB+delay_dist*math.sin(alpha)
	#error = desired_trajectory - CD
	error = -round(desired_trajectory - CD,2)
	#if f<2:
	#	vel = 0
	#print("A: ",a,"B: ",b,"Alfa: ", alpha, "Error: ", error)
	print("Alfa: ", alpha_deg, "Error: ", error)
	## END

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)
	

if __name__ == '__main__':
	#global safety_angle_deg
	print("Laser node started")
	print("Safety angle: ",safety_angle_deg)
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
