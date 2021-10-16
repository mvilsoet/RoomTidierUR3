#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	w = np.array([[0, 0, 1],[0, 1, 0],[0, 1, 0],[0, 1, 0],[1, 0, 0],[0, 1, 0]])

	Q = .001*np.array([[-150,150,162],[-150,270,162],[94,270,162],[(244+213-150),(270-93),162],[(244+213-150),270-10,162],[(244+213+83-150),270,162]])

	S_T = []
	for i in range(6):
		S_T.append(w[i])
		S_T.append(np.cross(-1*w[i], Q[i]))

	S_T = np.array(S_T).reshape(6,6)
	S = S_T.T

	#print(S)

	M = np.array([[0,-1,0,.390],[0,0,-1,.411],[1,0,0,.2155],[0,0,0,1]])

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Forward kinematics calculated:\n")

	M, S = Get_MS()
	inputs = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

	PoE = np.eye(4)

	for i in range(6):
		S_i = np.array([ [0,-S[2, i],S[1, i], S[3, i]] , [S[2, i],0,-S[0, i], S[4, i]] , [-S[1, i],S[0, i],0, S[5, i]] , [0,0,0,0] ])
			#            #0,-w3,     w2,      v1;         w3,     0,-w1,      v2;         -w2,     w1,     0, v3;         0
		PoE = np.matmul(PoE, expm(inputs[i]*S_i))

	PoE = np.matmul(PoE,M)
	return PoE

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	#cleanup
	yaw = yaw_WgripDegree * PI / 180
	L1 = .152
	L2 = .120
	L3 = .244
	L4 = .093
	L5 = .213
	L6 = .083
	L7 = .083
	L8 = .082
	L9 = .0535
	L10 = .059

	
	#base frame
	xgrip = xWgrip + .150
	ygrip = yWgrip - .150
	zgrip = zWgrip - .152

	#cen coor
	xcen = xgrip - L9 * np.sin(yaw)
	ycen = ygrip - L9 * np.cos(yaw)
	zcen = zgrip - L10

	theta_1 = math.atan2(ycen, xcen)

	#3end coor
	x3end = xcen - L7*np.sin(theta_1)-L6*np.cos(theta_1)
	y3end = ycen - L7*np.cos(theta_1)-L6*np.sin(theta_1)
	z3end = zcen + L10+L8

	d = (math.sqrt(x3end**2 + y3end**2) - L7)**2
	#thetas
	# theta_2 = -1*(atan2(L5,L3)+atan2(z3end-L1,sqrt((xend**2)+(yend**2))-L7))
	theta_2 = -1 * math.acos((2*(L3**2)*L5)/((z3end - L1)**2 + d - L3**2 - L5**2))
	theta_4 = math.asin((L1 + L3*np.sin(theta_2) - L8 - L10)/L5)
	theta_3 = -1*(theta_2 + theta_4)
	theta_5 = -PI/2
	theta_6 = theta_1 + PI/2 - yaw

	#double-check
	PoE = lab_fk(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
	print(PoE)
	print("Check forward kinematics P with input x,y,z:")
	print(np.array([[xWgrip], [yWgrip], [zWgrip]]))

	# ==============================================================#
	pass
   


