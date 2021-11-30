#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''
from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import math
import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
PI = pi
from lab2_header import *
from scipy.linalg import expm
from blob_search import *

vel = 4.0
accel = 4.0

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
midpt = [164*pi/180, -71.39*pi/180, 66.85*pi/180, -84*pi/180, -88.67*pi/180, 97.76*pi/180]
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

def input_callback(msg):

	global digital_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1

		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			rospy.loginfo("Goal is reached!")

		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

	"""
	start_xw_yw_zw: where to pick up a block in global coordinates
	target_xw_yw_zw: where to place the block in global coordinates
	"""
	start_xw_yw_zw = np.array(start_xw_yw_zw)
	target_xw_yw_zw = np.array(target_xw_yw_zw)
	start_xw_yw_zw = start_xw_yw_zw.flatten()
	# target_xw_yw_zw = target_xw_yw_zw.reshape(3,1)
	endabove = lab_invk(target_xw_yw_zw[0],target_xw_yw_zw[1],target_xw_yw_zw[2]+.1, 0)
	start = lab_invk(start_xw_yw_zw[0],start_xw_yw_zw[1],start_xw_yw_zw[2], 0)
	startabove = lab_invk(start_xw_yw_zw[0],start_xw_yw_zw[1],start_xw_yw_zw[2]+.1, 0)
	end = lab_invk(target_xw_yw_zw[0],target_xw_yw_zw[1],target_xw_yw_zw[2], 0)

	print("target: " , target_xw_yw_zw)
	print("start: " , start_xw_yw_zw)
	print("end: " , end)
	print("start: " , start)

	move_arm(pub_cmd, loop_rate, startabove, 4.0, 4.0)
	move_arm(pub_cmd, loop_rate, start, 4.0, 4.0)

	# while(True):
	# 	continue

	gripper(pub_cmd, loop_rate, suction_on)
	time.sleep(1.0)

	if(digital_in_0 == 1):
		move_arm(pub_cmd, loop_rate, startabove, 4.0, 4.0)
		move_arm(pub_cmd, loop_rate, endabove, 4.0, 4.0)
		move_arm(pub_cmd, loop_rate, end, 4.0, 4.0)

	if(digital_in_0 == 0):
		print("Block not found, continue")
	
	time.sleep(1)	
	gripper(pub_cmd, loop_rate, suction_off)
	move_arm(pub_cmd, loop_rate, endabove, 4.0, 4.0)

	return 1


def Get_MS():
	w = np.array([[0, 0, 1],[0, 1, 0],[0, 1, 0],[0, 1, 0],[1, 0, 0],[0, 1, 0]])

	Q = .001*np.array([[-150,150,162],[-150,270,162],[94,270,162],[(244+213-150),(270-93),162],[(244+213-150),270-10,162],[(244+213+83-150),270,162]])
		#convert mm to m

	S_T = []
	for i in range(6):
		S_T.append(w[i])
		S_T.append(np.cross(-1*w[i], Q[i]))

	S_T = np.array(S_T).reshape(6,6)
	S = S_T.T

	#print(S)

	M = np.array([[0,-1,0,.390],[0,0,-1,.411],[1,0,0,.2155],[0,0,0,1]])

	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Forward kinematics calculated:")

	M, S = Get_MS()
	inputs = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

	PoE = np.eye(4)

	for i in range(6):
		S_i = np.array([ [0,-S[2, i],S[1, i], S[3, i]] , [S[2, i],0,-S[0, i], S[4, i]] , [-S[1, i],S[0, i],0, S[5, i]] , [0,0,0,0] ])
			#            #0,-w3,     w2,      v1;         w3,     0,-w1,      v2;         -w2,     w1,     0, v3;         0
		PoE = np.matmul(PoE, expm(inputs[i]*S_i))

	PoE = np.matmul(PoE,M)
	T = PoE

	# print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	return_value = [0,0,0,0,0,0]
	L1 = .152
	L2 = .120
	L3 = .244
	L4 = .093
	L5 = .213
	L6 = .083
	L7 = .083
	L8 = .082
	L9 = 0.0535
	L10 = .059 

	x_grip = xWgrip+.150
	y_grip = yWgrip-.150
	z_grip = zWgrip-.010

	x_cen = x_grip-L9*np.cos(PI/180*yaw_WgripDegree)
	y_cen = y_grip-L9*np.sin(PI/180*yaw_WgripDegree)
	z_cen = z_grip

	bigtheta = np.arctan2(y_cen,x_cen)
	biglength = np.sqrt(x_cen**2 + y_cen**2)
	smalltheta = np.arcsin(.110/biglength)
	phi_1 = 180/PI*(bigtheta-smalltheta)

	phi_6 = phi_1+90-yaw_WgripDegree 

	x_3end = x_cen-.083*np.cos(PI/180*phi_1)+.110*np.sin(PI/180*phi_1)
	y_3end = y_cen-.110*np.cos(PI/180*phi_1)-.083*np.sin(PI/180*phi_1)
	z_3end = z_cen + L10 + L8

	L3end = np.sqrt(x_3end**2 + y_3end**2 + (z_3end-L1)**2)
	gamma1 = 180/PI*np.arccos((L3**2 + L5**2 - L3end**2)/(2*L3*L5))
	phi_3 = 180-gamma1

	gamma2 = 180/PI*np.arcsin((z_3end-L1)/L3end)
	gamma3 = 180/PI*np.arccos((L3**2 + L3end**2 - L5**2)/(2*L3*L3end))
	phi_2 = -(gamma2+gamma3)

	gamma4 = 180/PI*np.arccos((L5**2 + L3end**2 - L3**2)/(2*L5*L3end))
	phi_4 = -(gamma4 - gamma2)

	return_value[0] = np.deg2rad(phi_1) + PI
	return_value[1] = np.deg2rad(phi_2)
	return_value[2] = np.deg2rad(phi_3)
	return_value[3] = np.deg2rad(phi_4)-(0.5*PI)
	return_value[4] = np.deg2rad(-90)
	return_value[5] = np.deg2rad(phi_6)

	return return_value

class ImageConverter:

	def __init__(self, SPIN_RATE):

		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
		self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
		self.loop_rate = rospy.Rate(SPIN_RATE)

		# Check if ROS is ready for operation
		while(rospy.is_shutdown()):
			print("ROS is shutdown!")

	def image_callback(self, data):

		global xw_yw_G # store found green blocks in this list
		global xw_yw_P # store found purple blocks in this list

		try:
		  # Convert ROS image to OpenCV image
			raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv_image = cv2.flip(raw_image, -1)
		cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

		xw_yw_G = blob_search(cv_image, "green")
		xw_yw_P = blob_search(cv_image, "purple")    


def main():

	global home
	global Q
	global SPIN_RATE
	# Initialize ROS node
	rospy.init_node('lab2node')
	# Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

	ic = ImageConverter(SPIN_RATE)
	time.sleep(5)

	xw_yw_G_save = xw_yw_G
	xw_yw_P_save = xw_yw_P

	# xw_yw_G_save = xw_yw_G
	# xw_yw_P_save = xw_yw_P

	# block_choice = int(0)
	# while(True):
	# 	input_string = raw_input("Enter which object to yoink (1,2,3)")
	# 	print("You entered " + input_string + "\n")

	# 	input_string = int(input_string)
	# 	if(input_string < 0 or input_string > 4):
	# 		print("Pick block 1, 2, or 3, idiot")
	# 	else:
	# 		block_choice = int(input_string)
	# 		break



	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")
	rospy.loginfo("Sending Goals ...")
	loop_rate = rospy.Rate(SPIN_RATE)

	# while(True):
	# 	continue

	#TEMP
	i=1
	j=1

	rospy.loginfo("Sending goal 1 ...")
	for i in range(len(xw_yw_G_save)):
		print(xw_yw_G_save[i])
		if(xw_yw_G_save[i][1] > .3):
			print("??")
			dropoff_xyz = [.15,.1,.0318*i]
			move_block(pub_command, loop_rate, xw_yw_G_save[i], dropoff_xyz, vel, accel)
			i=i+1

	for j in range(len(xw_yw_P_save)):
		if(xw_yw_P_save[j][1] > .3):
			dropoff_xyz = [.15,.05,.0318*j]
			move_block(pub_command, loop_rate, xw_yw_P_save[j], dropoff_xyz, vel, accel)
			j=j+1

	gripper(pub_command, loop_rate, suction_off)
	move_arm(pub_command, loop_rate, go_away, 4.0, 4.0)

	sys.exit()


if __name__ == '__main__':

	try:
		main()
	# When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
