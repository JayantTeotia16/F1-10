#!/usr/bin/env python

import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive


kp = 30.0
kd = 0.5
prev_error = 0.0 
vel_input = 25.0
angle = 0	

pub = rospy.Publisher('car_1/multiplexer/command', AckermannDrive, queue_size = 10)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle

	error = data.pid_error
 	angle_correction = kp * error + (kd * (error - prev_error))
	angle = - angle_correction
	print angle
	if angle > 100:
		angle = 100
	elif angle < -100:
		angle = -100

	angle = angle / 100
	vel_input = 1 - abs(angle)
	if vel_input < 0.5:
		vel_input = 0.5
	msg = AckermannDrive();
	msg.speed = vel_input	
	msg.steering_angle = angle
	pub.publish(msg)

if __name__ == '__main__':
	global kp
	global kd
	global vel_input
	print("Listening to error for PID")
#	kp = input("Enter Kp Value: ")
#	kd = input("Enter Kd Value: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("/car_1/error", pid_input, control)
	rospy.spin()
