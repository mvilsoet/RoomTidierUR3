#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''
from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

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
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
Q11 = [149.59*pi/180, -46.45*pi/180, 101.8*pi/180, -147.25*pi/180, -91.4*pi/180, 62.38*pi/180]
Q12 = [152.19*pi/180, -52.77*pi/180, 96.79*pi/180, -131.36*pi/180, -89.85*pi/180, 85.8*pi/180]
Q13 = [151.84*pi/180, -57.43*pi/180, 94.25*pi/180, -124.19*pi/180, -90*pi/180, 85.48*pi/180]
Q21 = [166.59*pi/180, -48.37*pi/180, 101*pi/180, -140.28*pi/180, -88.92*pi/180, 100.16*pi/180]
Q22 = [166.91*pi/180, -54.53*pi/180, 101.16*pi/180, -135.28*pi/180, -88.67*pi/180, 100.52*pi/180]
Q23 = [166.9*pi/180, -59.78*pi/180, 99*pi/180, -127.86*pi/180, -88.65*pi/180, 100.53*pi/180] 
Q31 = [179.43*pi/180, -45.91*pi/180, 96.18*pi/180, -139.28*pi/180, -88.47*pi/180, 113*pi/180]
Q32 = [179.42*pi/180, -51.35*pi/180, 95.22*pi/180, -132.9*pi/180, -88.44*pi/180, 113*pi/180]
Q33 = [175.74*pi/180, -58.59*pi/180, 99.13*pi/180, -131.97*pi/180, -90.62*pi/180, 83.54*pi/180]
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
midpt = [164*pi/180, -71.39*pi/180, 66.85*pi/180, -84*pi/180, -88.67*pi/180, 97.76*pi/180]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q =np.array([ [Q11, Q12, Q13], [Q21, Q22, Q23], [Q31, Q32, Q33] ])
#Q[input_start][1]

############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN


############### Your Code End Here ###############


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


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".
    #Qxx -> Q[location][height]
    error = 0

    start_input = Q[start_loc-1][start_height-1]
    end_input = Q[end_loc-1][end_height-1]

    move_arm(pub_cmd, loop_rate, midpt, 4.0, 4.0) #go to mid

    move_arm(pub_cmd, loop_rate, start_input, 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.5)

        #CHECK HERE IF A BLOCK WAS ACTUALLY GRABBED

    time.sleep(0.5)
    if(digital_in_0 == 0):
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        print("Block not found, halting")
        sys.exit()
        return error

    move_arm(pub_cmd, loop_rate, midpt, 4.0, 4.0) #go to mid
    

    move_arm(pub_cmd, loop_rate, end_input, 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.5)

    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripped = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0
    input_start = 0
    input_end = 0
    input_mid = 0

    while(not input_done):
        input_string = raw_input("Enter tower starting location (1,2 or 3)")
        print("You entered " + input_string + "\n")
        if(int(input_string) == 1):
            input_start = 1
            input_done = 1

        if(int(input_string) == 2):
            input_start = 2
            input_done = 1

        if(int(input_string) == 3):
            input_start = 3
            input_done = 1
        if(int(input_string) == 0):
            sys.exit()
    input_done = 0

    while(not input_done):
        input_string = raw_input("Enter tower ending location (1,2, or 3)")
        print("You entered " + input_string + "\n")
        if(int(input_string) == 1):
            input_end = 1
            if input_start == 2:
                input_mid = 3
            if input_start == 3:
                input_mid = 2
            input_done = 1

        if(int(input_string) == 2):
            input_end = 2
            if input_start == 1:
                input_mid = 3
            if input_start == 3:
                input_mid = 1          
            input_done = 1

        if(int(input_string) == 3):
            input_end = 3
            if input_start == 1:
                input_mid = 2
            if input_start == 2:
                input_mid = 1
            input_done = 1
        if(int(input_string) == 0):
            sys.exit()   
    input_done = 0

    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

        #move_arm(pub_command, loop_rate, home, 4.0, 4.0)

        #rospy.loginfo("Sending goal 1 ...")
        #move_arm(pub_command, loop_rate, Q[input_start[0]][input_start[1]], 4.0, 4.0)

        #gripper(pub_command, loop_rate, suction_on)
        #gripper(pub_command, loop_rate, suction_off)
        # Delay to make sure suction cup has grasped the block
        #time.sleep(0.5)

        #def move_block(pub_cmd, loop_rate, start_loc, start_height, \
        #       end_loc, end_height):

    rospy.loginfo("Sending goal 1 ... begin tower")
    move_block(pub_command, loop_rate, input_start,3, input_end,1)
    rospy.loginfo("Sending goal 2 ...")
    move_block(pub_command, loop_rate, input_start,2, input_mid,1)
    rospy.loginfo("Sending goal 3 ...")
    move_block(pub_command, loop_rate, input_end,1, input_mid,2)
    rospy.loginfo("Sending goal 4 ...")
    move_block(pub_command, loop_rate, input_start,1, input_end,1)
    rospy.loginfo("Sending goal 5 ...")
    move_block(pub_command, loop_rate, input_mid,2, input_start,1)
    rospy.loginfo("Sending goal 6 ...")
    move_block(pub_command, loop_rate, input_mid,1, input_end,2)
    rospy.loginfo("Sending goal 7 ...")
    move_block(pub_command, loop_rate, input_start,1, input_end,3)

    gripper(pub_command, loop_rate, suction_off)
    move_arm(pub_command, loop_rate, midpt, 4.0, 4.0)


    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
