from __future__ import division
import Arm
from Arm import position, makeVector
from math import pi
import unittest

link1 = 1.0
link2 = 0.5

class InverseTestCase(unittest.TestCase):
    def test_case1(self):
        arm = Arm.Arm(link1=link1, link2=link2)

        end_effector = position(link1+link2, 0)
        joints = arm.inverse_kinematics(end_effector)
        self.assertAlmostEqual(joints.theta1, 0.0)
        self.assertAlmostEqual(joints.theta2, 0.0)

        end_effector = position(link1, link2)
        joints = arm.inverse_kinematics(end_effector)
        self.assertAlmostEqual(joints.theta1, 0.0)
        self.assertAlmostEqual(joints.theta2, pi / 2)

    def test_case2(self):
        arm = Arm.Arm(link1=link1, link2=link2)

        end_effector = position(link2, -link1)
        joints = arm.inverse_kinematics(end_effector)
        self.assertAlmostEqual(joints.theta1, -pi / 2)
        self.assertAlmostEqual(joints.theta2, pi / 2)

        end_effector = position(0, link1+link2)
        joints = arm.inverse_kinematics(end_effector)
        self.assertAlmostEqual(joints.theta1, pi / 2)
        self.assertAlmostEqual(joints.theta2, 0.0)

    def test_case3(self):
        arm = Arm.Arm(link1=link1, link2=link2, origin=makeVector(1,1))

        end_effector = position(1+link2, 1-link1)
        joints = arm.inverse_kinematics(end_effector)
        self.assertAlmostEqual(joints.theta1, -pi / 2)
        self.assertAlmostEqual(joints.theta2, pi / 2)

        end_effector = position(1, 1+link1+link2)
        joints = arm.inverse_kinematics(end_effector)
        self.assertAlmostEqual(joints.theta1, pi / 2)
        self.assertAlmostEqual(joints.theta2, 0.0)

    def test_unreachable(self):
        arm = Arm.Arm(link1=link1, link2=link2)
        end_effector = position(1+link1+link2, 0)
        with self.assertRaises(ValueError):
            joints = arm.inverse_kinematics(end_effector)

        end_effector = position(0, 0)
        with self.assertRaises(ValueError):
            joints = arm.inverse_kinematics(end_effector)

if __name__ == '__main__':
    unittest.main()
