#!/usr/bin/python

import numpy as np

'''	Exercise 3.46
	Write a function that returns "true" if given a 4x4 matrix is 
	within episilon of of an element of se(3) and false otherwise
'''

def function_exercise346(matrixT, episilon):
	wb = matrixT[np.ix_([0,1,2],[0,1,2])];
	
	is_so3 = np.all(abs(wb + wb.T) < episilon);
	
	is4x4 = matrixT.shape == (4,4)

	if(is4x4):
		bottomRowAllZero = matrixT[3,0] == 0 and matrixT[3,1] == 0 and matrixT[3,2] == 0 and matrixT[3,3] == 0
	else:
		bottomRowAllZero = False;	

	return (is4x4 and is_so3 and bottomRowAllZero);



def testharness():
	print('Testing wrong matrix size....')
	matrixT = np.identity(3);
	print(function_exercise346(matrixT, 1e-7))

	print('Testing identity matrix')
	matrixT = np.identity(4)
	print(function_exercise346(matrixT, 1e-7))

	print('Testing skew symetric but with (3,3) = 0')

	x1 = 2;
	x2 = 3;
	x3 = 4;

	matrixT[0,0] = 0;
	matrixT[0,1] = -x3;
	matrixT[0,2] = x2;
	
	matrixT[1,0] = x3;
	matrixT[1,1] = 0;
	matrixT[1,2] = -x1;

	matrixT[2,0] = -x2;
	matrixT[2,1] = x1;
	matrixT[2,2] = 0;

	matrixT[3,3] = 0;
	print(function_exercise346(matrixT, 1e-7))
