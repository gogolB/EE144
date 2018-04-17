#!/usr/bin/python

from  modern_robotics import *
import numpy as np

''' 	Exercise 3.44 from modern robots book
	Write a function that returns "True" if a given 4x4 Matrix is within
	episilon of an element of SE(3) and returns "false" otherwise.
'''

def function_exercise344(matrixT, episilon):
	
	R = matrixT[np.ix_([0,1,2], [0,1,2])]

	is_SO3 = np.all(abs(R.T*R - np.identity(3)) < episilon)
	is_4x4 = matrixT.shape == (4,4)
	is_det1 = np.linalg.det(R) - 1 < episilon

	if(is_4x4):
		bottomRowCorrect = matrixT[3,0] == 0 and matrixT[3,1] == 0 and matrixT[3,2] == 0 and matrixT[3,3] == 1
	else:
		bottomRowCorrect = False;

	return bottomRowCorrect and is_SO3 and is_4x4 and is_det1
	#return False;


def testharness():
	print('Testing a 3x3 identity matrix...')
	matrixT = np.identity(3);
	print(function_exercise344(matrixT,1e-7))	

	print('Testing a 4x4 identity matrix')
	matrixT = np.identity(4);
	print(function_exercise344(matrixT, 1e-7))

	# Other Test cases.
	print('Testing a 4x4 identity with (3,3) = 0')
	matrixT[3,3] = 0;
	print(function_exercise344(matrixT, 1e-7))
