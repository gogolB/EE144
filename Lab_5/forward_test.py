from __future__ import division
import Arm
from Arm import position, jointangle, makeVector
from math import pi, cos, sin
import unittest

link1 = 1.0
link2 = 0.5

class ForwardTestCase(unittest.TestCase):
	def test_case1(self):

		arm = Arm.Arm(link1=link1, link2=link2)

		end_effector = arm.compute_end_effector()
		self.assertAlmostEqual(end_effector.x, link1 + link2)
		self.assertAlmostEqual(end_effector.y, 0.0)

		joints = jointangle(0, -pi/2)
		end_effector = arm.forward_kinematics(joints)
		self.assertAlmostEqual(end_effector.x, link1 + link2 * cos(joints.theta2))
		self.assertAlmostEqual(end_effector.y, link2 * sin(joints.theta2))

		joints = jointangle(0, pi/4)
		end_effector = arm.forward_kinematics(joints)
		self.assertAlmostEqual(end_effector.x, link1 + link2 * cos(joints.theta2))
		self.assertAlmostEqual(end_effector.y, link2 * sin(joints.theta2))

	def test_case2(self):

		arm = Arm.Arm(link1=link1, link2=link2)

		joints = jointangle(pi/2, pi/2)
		end_effector = arm.forward_kinematics(joints)
		self.assertAlmostEqual(end_effector.x, -link2)
		self.assertAlmostEqual(end_effector.y, link1)

		joints = jointangle(-pi/2, pi/2)
		end_effector = arm.forward_kinematics(joints)
		self.assertAlmostEqual(end_effector.x, link2)
		self.assertAlmostEqual(end_effector.y, -link1)

	def test_case3(self):

		arm = Arm.Arm(link1=link1, link2=link2, origin=makeVector(1,1))

		joints = jointangle(pi/2, pi/2)
		end_effector = arm.forward_kinematics(joints)
		self.assertAlmostEqual(end_effector.x, 1 - link2)
		self.assertAlmostEqual(end_effector.y, 1 + link1)

		joints = jointangle(-pi/2, pi/2)
		end_effector = arm.forward_kinematics(joints)
		self.assertAlmostEqual(end_effector.x, 1 + link2)
		self.assertAlmostEqual(end_effector.y, 1 - link1)


if __name__ == '__main__':
	unittest.main()
