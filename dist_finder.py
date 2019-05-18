#!/usr/bin/env python

import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

desired_trajectory = 0.5        # Keep the car at this distance from the right wall
vel_input = 35                  # Chosen speed
vel = 35                        # Actual speed of the car
delay_dist = vel/100            # Distance the car travels while
safety_dist = 1                 # Stop if something is within 1 meter in front of the car
car_width = 0.4                 # Car width(0.3) + margin
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
	theta = 50              # Angle to scan wall
	prev_vel = vel          # Previous velocity
	vel = vel_input         # Set velocity to chosen speed
	a = getRange(data,theta)
	b = getRange(data,0)
	swing = math.radians(theta)
	
	## Your code goes here
	# Stop car if something in front of it.
	
	for x in numpy.arange(math.floor(90-safety_angle_deg), math.ceil(90+safety_angle_deg),1):
		dist = getRange(data,int(round(x)))	
		if (dist*math.sin(math.radians(x))<safety_dist and dist!=-1):
			vel = 0 # Stop car if something is in front of it (car_width)
			print("STOP: ",dist,"Angle: ",x)
	if (vel!=0 and prev_vel == 0):
		vel = vel_input # Start driving (at vel_input) if nothing in front of car
	
	alpha=round(math.atan((a*math.cos(swing)-b)/(a*math.sin(swing))),2)
	alpha_deg=round(math.degrees(alpha),2)
	AB=b*math.cos(alpha)
	CD=AB+delay_dist*math.sin(alpha)
	error = -round(desired_trajectory - CD,2)
	print("Alfa: ", alpha_deg, "Error: ", error)
	## END

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)
	

if __name__ == '__main__':
        global vel_input
	print("Laser node started")
	vel_input = input("Enter Velocity: ")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
