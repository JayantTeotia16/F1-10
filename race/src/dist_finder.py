#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input



angle_range = 0		# sensor angle range of the lidar
car_length = 1.5	# distance (in m) that we project the car forward for correcting the error. 
desired_trajectory = 2	# distance from the wall (left or right - we cad define..but this is defined for right).
vel = 20 		
error = 0.0
kp = 1
kd = 0.05
pub = rospy.Publisher('/car_1/error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##		theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta

def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
# Return the lidar scan value at that index
# Do some error checking for NaN and ubsurd values
## Your code goes here
	
	index = int(math.radians(45 + theta) * (1 / data.angle_increment))
	if data.ranges[index] >= data.range_min and data.ranges[index] <= data.range_max:
		return data.ranges[index]
	return 0.1 
def callback(data):
	theta = 50;
	a = getRange(data,theta) 
	b = getRange(data,0)	
	swing = math.radians(theta)
	
	## Your code goes here to compute alpha, AB, and CD..and finally the error.

	alpha = math.atan2((a*math.cos(swing) - b) , (a * math.sin(swing)))
	ab = b * math.cos(alpha)
	cd = ab + (car_length * math.sin(alpha))
	error = cd - desired_trajectory
	print error


	## END

	msg = pid_input()
	msg.pid_error = error		
	msg.pid_vel = vel		
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("/car_1/scan",LaserScan,callback)
	rospy.spin()
