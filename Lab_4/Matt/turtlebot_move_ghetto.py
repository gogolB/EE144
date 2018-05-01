#!/usr/bin/env python

import rospy
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
	
		# Move forward at 0.5 m/s
		self.vel = Twist()
		self.vel.linear.x = 0.5
		self.vel.angular.z = 0

		# Rotate in Place at 90 Degrees Per second.
		self.rotate_vel = Twist()
		self.rotate_vel.linear.x = 0
		self.rotate_vel.angular.z = 1.57075

		# Set update rate at 10 Hz
		self.rate = rospy.Rate(10);

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

	def makeSquare(self):
		while True:
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
				rospy.loginfo("position: "+str(position))
				rospy.loginfo("orientation: "+str(orientation))
			except:
				continue;

			# Move forward for 50 time ticks. (0.5 m/s * 10 seconds * 10 ticks per second)
			for i in range(50):
				self.setVel(self.vel)
				self.sleep()
			rospy.loginfo("Moved 5 M...")

			# Rotate in place for 10 time ticks.
			for i in range(10):
				self.setVel(self.rotate_vel)
				self.sleep()
			rospy.loginfo("Turned 90 Deg...")

	def makeSquareP(self):
		phi_goal = 0
		posx_goal = 0
		posy_goal = 0
		pos_thresh = 0.1
		phi_thresh = 0.1
		phi_str_const = 8
		phi_turn_const = 2
		self.curr_vel = Twist()
		self.curr_vel.linear.x = 0.5

		while True:
			#get init position
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
				#rospy.loginfo("position: "+str(position))
				#rospy.loginfo("orientation: "+str(orientation))
			except:
				continue
			#set x and y goal 
			posx_goal = 5
			while (abs(posx_goal - position[0]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_str_const * (phi_goal - orientation[2])
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
			
			#set new rotation goal
			phi_goal = 1.570796
			
			while (abs(phi_goal - orientation[2]) > phi_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					#rospy.loginfo("position: "+str(position))
					#rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_turn_const * (phi_goal - orientation[2])
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
				
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
				
			#set new pos goal and set lin vel
			posy_goal = 5
			self.curr_vel.linear.x = 0.5
			
			while (abs(posy_goal - position[1]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_str_const * (phi_goal - orientation[2])
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
			
			#set new rotation goal
			phi_goal = 2*1.570796
			
			while (abs(phi_goal - orientation[2]) > phi_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_turn_const * (phi_goal - orientation[2])
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
				
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
			
			posx_goal = 0
			phi_goal = 2*1.570796
			self.curr_vel.linear.x = 0.5
			while (abs(posx_goal - position[0]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				if(orientation[2] > 0 and orientation[2] < 2*1.570796):
					phi_error = phi_str_const/2 * (phi_goal - orientation[2])
					self.curr_vel.angular.z = phi_error
					self.setVel(self.curr_vel)
					self.sleep()
				else:
					phi_error = -1 * phi_str_const * (phi_goal - orientation[2])
					self.curr_vel.angular.z = phi_error
					self.setVel(self.curr_vel)
					self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
				
			#set new rotation goal
			phi_goal = -1.570796
			
			while (abs(phi_goal - orientation[2]) > phi_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_turn_const * (phi_goal - orientation[2])
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
				
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
				
			#set new pos goal and set lin vel
			posy_goal = 0
			self.curr_vel.linear.x = 0.5
			
			while (abs(posy_goal - position[1]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_str_const * (phi_goal - orientation[2])
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			break
	
	def makeSquarePD(self):
		phi_goal = 0
		posx_goal = 0
		posy_goal = 0
		pos_thresh = 0.1
		phi_thresh = 0.1
		phi_str_const = 8
		phi_str_constD = 4
		phi_error_old = 0
		phi_turn_const = 2
		self.curr_vel = Twist()
		self.curr_vel.linear.x = 0.5

		while True:
			#get init position
			try:
				(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				orientation = tf.transformations.euler_from_quaternion(quaternion)
				#rospy.loginfo("position: "+str(position))
				#rospy.loginfo("orientation: "+str(orientation))
			except:
				continue
			#set x and y goal 
			posx_goal = 5
			while (abs(posx_goal - position[0]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_str_const * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
				phi_error_old = orientation[2]
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
			
			#set new rotation goal
			phi_goal = 1.570796
			
			while (abs(phi_goal - orientation[2]) > phi_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					#rospy.loginfo("position: "+str(position))
					#rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_turn_const * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
				phi_error_old = orientation[2]
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
				
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
				
			#set new pos goal and set lin vel
			posy_goal = 5
			self.curr_vel.linear.x = 0.5
			
			while (abs(posy_goal - position[1]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_str_const * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
				phi_error_old = orientation[2]
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
			
			#set new rotation goal
			phi_goal = 2*1.570796
			
			while (abs(phi_goal - orientation[2]) > phi_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_turn_const * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
				phi_error_old = orientation[2]
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
				
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
			
			posx_goal = 0
			phi_goal = 2*1.570796
			self.curr_vel.linear.x = 0.5
			while (abs(posx_goal - position[0]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				if(orientation[2] > 0 and orientation[2] <= 2*1.570796):
					phi_error = phi_str_const/2 * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
					phi_error_old = orientation[2]
					self.curr_vel.angular.z = phi_error
					self.setVel(self.curr_vel)
					self.sleep()
				else:
					phi_error = -1 * phi_str_const/2 * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
					phi_error_old = orientation[2]
					self.curr_vel.angular.z = phi_error
					self.setVel(self.curr_vel)
					self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
				
			#set new rotation goal
			phi_goal = -1.570796
			
			while (abs(phi_goal - orientation[2]) > phi_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_turn_const * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
				phi_error_old = orientation[2]
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
				
			#allow some time to settle
			for i in range(10):
				self.curr_vel.linear.x = 0
				self.curr_vel.angular.z = 0
				self.setVel(self.curr_vel)
				self.sleep()
				
			#set new pos goal and set lin vel
			posy_goal = 0
			self.curr_vel.linear.x = 0.5
			
			while (abs(posy_goal - position[1]) > pos_thresh):
				try:
					(position, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
					orientation = tf.transformations.euler_from_quaternion(quaternion)
					rospy.loginfo("position: "+str(position))
					rospy.loginfo("orientation: "+str(orientation))
				except:
					continue
				phi_error = phi_str_const * (phi_goal - orientation[2]) + phi_str_constD * (phi_error_old - orientation[2])
				phi_error_old = orientation[2]
				self.curr_vel.angular.z = phi_error
				self.setVel(self.curr_vel)
				self.sleep()
			#rospy.loginfo("Moved 5 M...")
			#set robot to stop
			self.curr_vel.linear.x = 0
			self.curr_vel.angular.z = 0
			self.setVel(self.curr_vel)
			break

if __name__ == '__main__':
	try:
		tb = turtlebot_move()
		tb.makeSquarePD()
	except rospy.ROSInterruptException:
         rospy.loginfo("Action terminated.")