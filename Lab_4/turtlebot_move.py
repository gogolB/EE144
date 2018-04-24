#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from time import time
import tf
from geometry_msgs.msg import Twist
import numpy as np;

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
		self.rate = rospy.Rate(100);

		self.sideLength = 5;
		self.pos = None;
		self.rot = None;                     
        
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
			for i in range(500):
				self.setVel(self.vel)
				self.sleep()
			rospy.loginfo("Moved 5 M...")

			# Rotate in place for 10 time ticks.
			for i in range(100):
				self.setVel(self.rotate_vel)
				self.sleep()
			rospy.loginfo("Turned 90 Deg...")

	def getPos(self):
		try:
			(self.pos, quaternion) = self.tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
			self.rot = tf.transformations.euler_from_quaternion(quaternion)
			if(self.rot[2] < 0):
				newZ = 3.1415 + (3.1415 - abs(self.rot[2]))
				self.rot = (self.rot[0],self.rot[1],newZ)
		except:
			return

	def makeSquareP(self):
		# First get orientation
		self.getPos();
		while(self.pos is None or self.rot is None):
			self.getPos();
		
		print("Pos: " + str(self.pos))
		print("Orentation: " + str(self.rot));
		
		desiredOrientation = self.rot[2];
		desiredPosition = np.array(self.pos, copy=True);
		desiredPosition[0] = desiredPosition[0] + self.sideLength;
		print("Desired Position: " + str(desiredPosition))
		
		kp = 2;

		# Move to new location
		while(abs(desiredPosition[0] - self.pos[0]) > 1e-2):
			#print("Distance to Go: " + str(abs(desiredPosition[0] - self.pos[0])))
			w = kp * (desiredOrientation - self.rot[2]);
			vel = Twist()
			vel.linear.x = 0.5
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();

		stop_vel = Twist()
		stop_vel.linear.x = 0
		stop_vel.angular.z = 0
		self.set_velocity.publish(stop_vel)	

		print("Error: " + str(abs(desiredPosition[0] - self.pos[0])))
		
		desiredOrientation = self.rot[2] + 3.151529/2;
		print("Desired Orientation: " + str(desiredOrientation))

		# Turn 90 degrees
		while(abs(desiredOrientation - self.rot[2]) > 1e-2):
			#print("Rotation to Go: " + str(abs(desiredOrientation - self.rot[2])))
			w = kp * (desiredOrientation - self.rot[2]);
			vel = Twist()
			vel.linear.x = 0
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();	

		self.getPos();		
		print("Pos: " + str(self.pos))
		print("Orentation: " + str(self.rot));
		
		print("Error: " + str(abs(desiredOrientation - self.rot[2])))
		
		# Move to Location 2
		desiredOrientation = self.rot[2];
		desiredPosition = np.array(self.pos, copy=True);
		desiredPosition[1] = desiredPosition[1] + self.sideLength;
		print("Desired Position 2: " + str(desiredPosition))

		while(abs(desiredPosition[1] - self.pos[1]) > 1e-2):
			#print("Distance to Go: " + str(abs(desiredPosition[1] - self.pos[1])))
			w = kp * (desiredOrientation - self.rot[2]);
			vel = Twist()
			vel.linear.x = 0.5
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();

		stop_vel = Twist()
		stop_vel.linear.x = 0
		stop_vel.angular.z = 0
		self.set_velocity.publish(stop_vel)	

		print("Error: " + str(abs(desiredPosition[1] - self.pos[1])))

		desiredOrientation = self.rot[2] + 3.151529/2;
		print("Desired Orientation: " + str(desiredOrientation))

		# Turn 90 degrees
		while(abs(desiredOrientation - self.rot[2]) > 1e-1):
			print("Rotation to Go: " + str(abs(desiredOrientation - self.rot[2])))
			w = kp * (desiredOrientation - self.rot[2]);
			vel = Twist()
			vel.linear.x = 0
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();	

		self.getPos();		
		print("Pos: " + str(self.pos))
		print("Orentation: " + str(self.rot));
		
		print("Error: " + str(abs(desiredOrientation - self.rot[2])))
		

		# Move to Location 3		
		desiredOrientation = self.rot[2];
		desiredPosition = np.array(self.pos, copy=True);
		desiredPosition[0] = desiredPosition[0] - self.sideLength;
		print("Desired Position 3: " + str(desiredPosition))
		while(abs(desiredPosition[0] - self.pos[0]) > 1e-2):
			print("Rotation adjust: " + str(desiredOrientation - self.rot[2]))
			w = kp * (desiredOrientation - self.rot[2]);

			vel = Twist()
			vel.linear.x = 0.5
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();

		stop_vel = Twist()
		stop_vel.linear.x = 0
		stop_vel.angular.z = 0
		self.set_velocity.publish(stop_vel)	

		print("Error: " + str(abs(desiredPosition[0] - self.pos[0])))
		
		desiredOrientation = self.rot[2] + 3.151529/2;
		print("Desired Orientation: " + str(desiredOrientation))

		# Turn 90 degrees
		while(abs(desiredOrientation - self.rot[2]) > 1e-1):
			print("Rotation to Go: " + str(abs(desiredOrientation - self.rot[2])))
			w = kp * (desiredOrientation - self.rot[2]);
			vel = Twist()
			vel.linear.x = 0
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();	

		self.getPos();		
		print("Pos: " + str(self.pos))
		print("Orentation: " + str(self.rot));
		
		print("Error: " + str(abs(desiredOrientation - self.rot[2])))
		
		# Move to Location 4
		desiredOrientation = self.rot[2];
		desiredPosition = np.array(self.pos, copy=True);
		desiredPosition[1] = desiredPosition[1] - self.sideLength;
		print("Desired Position 2: " + str(desiredPosition))

		while(abs(desiredPosition[1] - self.pos[1]) > 1e-2):
			print("Distance to Go: " + str(abs(desiredPosition[1] - self.pos[1])))
			w = kp * (desiredOrientation - self.rot[2]);
			vel = Twist()
			vel.linear.x = 0.5
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();

		stop_vel = Twist()
		stop_vel.linear.x = 0
		stop_vel.angular.z = 0
		self.set_velocity.publish(stop_vel)	

		print("Error: " + str(abs(desiredPosition[1] - self.pos[1])))

		desiredOrientation = self.rot[2] + 3.151529/2;
		print("Desired Orientation: " + str(desiredOrientation))

		# Turn 90 degrees
		while(abs(desiredOrientation - self.rot[2]) > 1e-2):
			print("Rotation to Go: " + str(abs(desiredOrientation - self.rot[2])))
			w = kp * (desiredOrientation - self.rot[2]);
			vel = Twist()
			vel.linear.x = 0
			vel.angular.z = w
			
			self.setVel(vel);			
			self.sleep();
			self.getPos();	

		self.getPos();		
		print("Pos: " + str(self.pos))
		print("Orentation: " + str(self.rot));
		
		print("Error: " + str(abs(desiredOrientation - self.rot[2])))	
			
			
if __name__ == '__main__':
	try:
		tb = turtlebot_move()
		tb.makeSquareP()
	except rospy.ROSInterruptException:
		rospy.loginfo("Action terminated.")

