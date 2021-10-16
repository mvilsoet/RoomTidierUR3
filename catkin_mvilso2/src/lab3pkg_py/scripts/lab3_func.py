#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix

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

	# =========== Implement joint angle to encoder expressions here ===========
	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	M, S = Get_MS()
	inputs = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

	PoE = np.eye(4)

	for i in range(6):
		S_i = np.array([ [0,-S[2, i],S[1, i], S[3, i]] , [S[2, i],0,-S[0, i], S[4, i]] , [-S[1, i],S[0, i],0, S[5, i]] , [0,0,0,0] ])
			# 			 #0,-w3,     w2,      v1;         w3,     0,-w1,      v2;         -w2,     w1,     0, v3;         0
		PoE = np.matmul(PoE, expm(inputs[i]*S_i))

	PoE = np.matmul(PoE,M)
	T = PoE

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
