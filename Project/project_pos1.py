#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Empty
from time import time
import tf
from geometry_msgs.msg import Twist

class turtlebot_move():
	def __init__(self):
		rospy.init_node('turtlebot_move', anonymous=False)

		rospy.loginfo("Press CTRL + C to terminate")
		rospy.on_shutdown(self.shutdown)
		
		self.reset_odom = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10)
		self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

		timer=time()
		while time()-timer<1.0:
			self.reset_odom.publish(Empty())
	
		self.tfListener = tf.TransformListener();

		# Set update rate at 50 Hz
		self.rate = rospy.Rate(50);

	def shutdown(self):
		rospy.loginfo("Stop Action")
		stop_vel = Twist()
		stop_vel.linear.x = 0
		stop_vel.angular.z = 0
		self.set_velocity.publish(stop_vel)
		rospy.sleep(1)

	def setVel(self, twist):
		self.set_velocity.publish(twist);

	def sleep(self):
		self.rate.sleep()
		
	def distance(self,x2,y2,x1,y1):
		return np.sqrt((x2-x1)**2 + (y2-y1)**2);

	def angle(self, x2,y2,x1,y1):
		y = y2-y1;
		x = x2-x1;
		return np.arctan2(y,x);

	def kickgoal_pos1(self):
		#setup linear/angular velocity goals, and p controller constants
		phi_vel_goal = -1 #rads/s clockwise
		lin_vel_goal = 0.5 #m/s
		phi_vel_p = 1
		lin_vel_p = 1.1

		#variables for p controllers
		last_pos = [0, 0]
		curr_pos = [0, 0]
		actual_lin_vel = 0
		last_ang = 0
		curr_ang = 0
		actual_phi_vel = 0;

		#setup initial stationary twist
		self.curr_vel = Twist()
		self.curr_vel.linear.x = 0.0
		self.curr_vel.angular.z = 0.0
		self.setVel(self.curr_vel)

		#get init position until it works
		while True:
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
				break
			except:
				self.sleep()
				continue
		#set initial twist
		self.curr_vel.linear.x = lin_vel_goal
		self.curr_vel.angular.z = phi_vel_goal
		self.setVel(self.curr_vel)
		
		#FIXME: might need some time to let motors ramp up before data is usable

		#run p controller in loop
		while True:
			#get current position and orientation
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
			except:
				continue
			
			#set old position and orientation variables to previously used current values
			last_pos = [curr_pos[0], curr_pos[1]]
			last_ang = curr_ang
			
			#read then set current position and orientation variables
			curr_pos = [position[0], position[1]]
			curr_ang = orientation[2]
			
			#calculate actual linear velocity = change in distance/time
			actual_lin_vel = self.distance(curr_pos[0], curr_pos[1], last_pos[0], last_pos[1])*50
			#print("Actual linear:")
			#print(actual_lin_vel)
			
			#calculate actual angular velocity = change in angle/time
			actual_phi_vel = (curr_ang - last_ang)*50
			#print("Actual angular:")
			#print(actual_phi_vel)
			
			#print("Trying to set linear: ")
			#print((lin_vel_p*(lin_vel_goal - actual_lin_vel)))
			
			#print("Trying to set angular: ")
			#print((phi_vel_p*(phi_vel_goal - actual_phi_vel)))
			print("%7.5f,%7.5f,%7.5f,%7.5f,%7.5f"%(actual_lin_vel,actual_phi_vel,(lin_vel_p*(lin_vel_goal - actual_lin_vel)),(phi_vel_p*(phi_vel_goal - actual_phi_vel)), self.distance(curr_pos[0], curr_pos[1],0,0)));
			
			#set new lin and ang velocities with p controller
			self.curr_vel.linear.x = lin_vel_p*(lin_vel_goal - actual_lin_vel)
			self.curr_vel.angular.z = phi_vel_p*(phi_vel_goal - actual_phi_vel)
			self.setVel(self.curr_vel)
			
			#sleep for 1/50 seconds
			self.sleep()

if __name__ == '__main__':
	try:
		tb = turtlebot_move()
		tb.kickgoal_pos1()
	except rospy.ROSInterruptException:
         rospy.loginfo("Action terminated.")