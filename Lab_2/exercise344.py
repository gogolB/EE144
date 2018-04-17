#!/usr/bin/python

from  modern_robotics import *
import numpy as np

''' 	Exercise 3.44 from modern robots book
	Write a function that returns "True" if a given 4x4 Matrix is within
	episilon of an element of SE(3) and returns "false" otherwise.
'''

def function_exercise344(matrixT, episilon):
	
	R = matrixT[np.ix_([0,1,2], [0,1,2])]

	return np.all(abs(R.T*R - np.identity(3)) < episilon) 
	
	#return False;


def testharness():
	matrixT = np.identity(4);
	print(function_exercise344(matrixT, 1e-7))
