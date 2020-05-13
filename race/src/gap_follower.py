#!/usr/bin/env python

import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
import numpy as np
import math

pub = rospy.Publisher('/car_1/multiplexer/command', AckermannDrive, queue_size = 10)

car_width = 0.25
angle_inflate = 0

def angle_crop(data):
	forward_scan = np.zeros(int(np.pi * (1/data.angle_increment)))
	for i in range(len(forward_scan)):
		indice = int((np.pi/4) * (1/data.angle_increment) + i)
		forward_scan[i] = data.ranges[indice]
	return forward_scan

def obstacle_detect(forward_scan):
	obs_detect = np.zeros(len(forward_scan) - 1)
	for i in range(len(obs_detect)):
		obs_detect[i] = forward_scan[i+1] - forward_scan[i]
		if forward_scan[i] < 0.2:
			forward_scan[i] = (forward_scan[i + 1] + forward_scan[i + 2]) / 2
		if obs_detect[i] <= 0.5:
			obs_detect[i] = 0
	return obs_detect

def inflate_func(obs_detect, forward_scan):
	global angle_inflate
	for i in range(len(obs_detect)):
		if obs_detect[i] > 0:
			for j in range(i + 1, i + angle_inflate):
				if j > len(obs_detect):
					break
				forward_scan[j] = forward_scan[i]
		elif obs_detect[i] < 0:
			for j in range(i - angle_inflate, i):
				if j <0:
					j = 0
				forward_scan[j] = forward_scan[i+1]
	return forward_scan

def calculate_angle(data, forward_scan, max_free_space):
	angle = (len(forward_scan) / 2) * data.angle_increment - max_free_space * data.angle_increment
	angle = -1 * angle / (100 * np.pi / 180) 
	return angle

def control_function(angle):
	kpv = 1
	kps = 2
	velocity = kpv * (1 - abs(angle))
	if velocity < 0.4:
		velocity = 0.4
	angle = kps * (angle)
	if angle > 1:
		angle = 1
	elif angle < -1:
		angle = -1
	return velocity, angle

def callback(data):
	global car_width, angle_inflate
	forward_scan = angle_crop(data) 
	obs_detect = obstacle_detect(forward_scan) 
	closest_obstacle = min(forward_scan)
	closest_obstacle_index = np.argmin(forward_scan)	
	angle_inflate = int((car_width / closest_obstacle) * (1 / data.angle_increment))
	forward_scan = inflate_func(obs_detect, forward_scan)
	max_free_space = np.argmax(forward_scan)	
	angle = calculate_angle(data, forward_scan, max_free_space) 
	velocity, angle = control_function(angle)
	print angle
	msg = AckermannDrive();
	msg.speed = velocity	
	msg.steering_angle = angle
	pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('gap_follower', anonymous=True)
	rospy.Subscriber("/car_1/scan", LaserScan, callback)
	rospy.spin()
