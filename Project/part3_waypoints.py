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

		# Set update rate at 1000 Hz
		self.rate = rospy.Rate(100);

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
			
	def gotowaypoint(self, x_pnt, y_pnt):
		#1. Get required angle
		#2. Use PD to align 
		#3. Use PD to follow to waypoint
		#4. Check current distance often
		self.phi_goal = 0
		self.pos_thresh = 0.1
		self.phi_thresh = np.pi/32
		self.phi_str_const = 1
		self.phi_turn_const = 0.45
		self.curr_vel = Twist()
		self.curr_vel.linear.x = 0.0
		self.curr_vel.angular.z = 0.0
		
		while True:
			#get init position
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				self.orientation = tf.transformations.euler_from_quaternion(quaternion)
				#rospy.loginfo("position: "+str(position))
				#rospy.loginfo("orientation: "+str(orientation))
				break
			except:
				self.sleep()
				continue
		
		#get required angle and init dist
		self.phi_goal = self.angle(x_pnt, y_pnt, position[0], position[1])
		if(self.distance(x_pnt, y_pnt, position[0], position[1]) < self.pos_thresh):
			return
		
		#run P to align
		self.turntowaypnt(x_pnt, y_pnt)
		#set robot to stop
		self.curr_vel.linear.x = 0
		self.curr_vel.angular.z = 0
		self.setVel(self.curr_vel)
		#allow some time to settle
		for i in range(5):
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			self.sleep()

		#run while checking current distance
		self.gostraight(x_pnt, y_pnt)
		#set robot to stop
		self.curr_vel.linear.x = 0
		self.curr_vel.angular.z = 0
		self.setVel(self.curr_vel)
		#allow some time to settle
		for i in range(5):
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			self.sleep()
	
	def distance(self,x2,y2,x1,y1):
		return np.sqrt((x2-x1)**2 + (y2-y1)**2);

	def angle(self, x2,y2,x1,y1):
		y = y2-y1;
		x = x2-x1;
		return np.arctan2(y,x);
	
	def turntowaypnt(self, x_pnt, y_pnt):
		while True:
			#get init position
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
				#rospy.loginfo("position: "+str(position))
				#rospy.loginfo("orientation: "+str(orientation))
				break
			except:
				self.sleep()
				continue
			
		#print(self.distance(x_pnt, y_pnt, position[0], position[1]))
		if(self.distance(x_pnt, y_pnt, position[0], position[1]) < self.pos_thresh):
			return
		
		self.phi_goal = self.angle(x_pnt, y_pnt, position[0], position[1])
		#print(self.phi_goal)
		while (abs(self.phi_goal - orientation[2]) > self.phi_thresh):
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
				#rospy.loginfo("position: "+str(position))
				#rospy.loginfo("orientation: "+str(orientation))
			except:
				self.sleep()
				continue
			
			print("turning towards %.2f %.2f|e: %.2f|g:%.2f" %(x_pnt, y_pnt,(self.phi_goal - orientation[2]), self.phi_goal))
			
			self.phi_error = self.phi_turn_const * (self.phi_goal - orientation[2])
			#print(orientation[2])
			self.phi_error = self.phi_error % np.pi
			self.curr_vel.angular.z = self.phi_error
			#print("+")
			#print(abs(phi_goal - orientation[2]))
			self.setVel(self.curr_vel)
			self.sleep()
				
	def gostraight(self, x_pnt, y_pnt):
		while True:
			#get init position
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
				rospy.loginfo("position: "+str(position))
				#rospy.loginfo("orientation: "+str(orientation))
				break
			except:
				self.sleep()
				continue
			
		
		while (self.distance(x_pnt, y_pnt, position[0], position[1]) > self.pos_thresh):
			print(self.distance(x_pnt, y_pnt, position[0], position[1]))
			self.curr_vel.linear.x = 0.5
			
			try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					#rospy.loginfo("orientation: "+str(orientation))
			except:
				continue
			print("straight towards %f %f" %(x_pnt, y_pnt));
			#self.phi_goal = self.angle(x_pnt, y_pnt, position[0], position[1])
			
			self.curr_vel.angular.z = (self.phi_goal - orientation[2]);
			self.setVel(self.curr_vel)
			self.sleep();
				

if __name__ == '__main__':
	#FIXME- the robot turns to pi/2 when it reaches 1,0
	waypoints = [[0.707,0],[0.707,0.9],[0,0]]
	#,[1,1],[1,1],[0.5,1],[0,1],[0,1],[0,0.5],[0,0]]
	try:
		tb = turtlebot_move()
		for p in waypoints:
			tb.gotowaypoint(p[0],p[1])
					
	except rospy.ROSInterruptException:
         rospy.loginfo("Action terminated.")
