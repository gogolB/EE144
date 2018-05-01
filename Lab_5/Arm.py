from __future__ import division
from math import pi
import numpy as np
from collections import namedtuple
from scipy.linalg import expm

EPSILON = 1e-3

position = namedtuple('position', ['x', 'y'])
jointangle = namedtuple('jointangle', ['theta1', 'theta2'])
makeVector = namedtuple('makeVector', ['x', 'y'])

class Arm(object):
		#q0 - initial positions of joints
		#origin - position of the base of the arm in carthesian space
	def __init__(self, link1=1.0, link2=1.0, q0=jointangle(0,0), origin=makeVector(0,0)):
		self.link1 = link1
		self.link2 = link2
		self.lsq = link1 ** 2 + link2 ** 2
		self.joints = q0
		self.origin = origin
		self.end_effector = self.compute_end_effector()

	def forward_kinematics(self, input_joints):
		self.joints = input_joints
		self.end_effector = self.compute_end_effector()
		return self.end_effector

	def compute_end_effector(self):
	#the return of the function is the position of end_effector, start with self.joints.theta1 and self.joints.theta2
		################################ Computes end_effector position knowing joint angles, your code goes between ##############################
		#print("-----")
		M = np.identity(4)
		M[0,3] =  self.link1 + self.link2 + self.origin.x;
		M[1,3] =  self.origin.y;
		
		
		s1_brackets = np.matrix([[0, -1, 0, +self.origin.y],
								 [1,  0, 0, -self.origin.x],
								 [0,  0, 0, 0],
								 [0,  0, 0, 0]]);
								 
		s2_brackets = np.matrix([[0, -1, 0, +self.origin.y],
								 [1,  0, 0, -self.link1 - self.origin.x],
								 [0,  0, 0, 0],
								 [0,  0, 0, 0]]);
								 
		s1_brackets_theta1 = s1_brackets * self.joints.theta1
		s2_brackets_theta2 = s2_brackets * self.joints.theta2
		
		T = np.matrix(expm(s1_brackets_theta1)) * np.matrix(expm(s2_brackets_theta2)) * np.matrix(M)
		#print("S1, S2, M, T");
		#print(expm(s1_brackets_theta1))
		#print(expm(s2_brackets_theta2))
		#print(M);
		#print(T);
		
		x = T[0,3]
		y = T[1,3]
	###########################################################################################################################################
		return position(x, y)

	def inverse_kinematics(self, input_ee):
	############################### check if the end effector position is reachable, your code goes below #####################################
	#in your code, please include 
	#raise ValueError('your words')
	#so that your code can pass the test case
		r = input_ee.x ** 2 + input_ee.y ** 2
		rmin = self.link1 ** 2 - self.link2 ** 2
		rmax = self.link1 ** 2 + self.link2 ** 2
		r = sqrt(r);
		rmin = sqrt(rmin)
		rmax = sqrt(rmax)
		if(r < rmin or r > rmax):
			raise ValueError('invalid input. Not possible to reach')

		self.end_effector = input_ee
		self.joints = self.compute_joints()
		return self.joints

	def compute_joints(self):
	#the return of the function are angles of joints, which should stay between -pi and pi. Start with self.end_effector.x and self.end_effector.y.
		################################# Computes joint angle knowing end effector position, your code goes below #################################



		theta1 = 0;
		theta2 = 0
		return jointangle(theta1, theta2)

