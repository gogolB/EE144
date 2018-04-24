#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class turtlebot_move():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)

	rospy.loginfo("Press CTRL + C to terminate")
 
        rospy.on_shutdown(self.shutdown)
        
        self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

	# Move forward at 0.5 m/s
        vel = Twist()
        vel.linear.x = 0.5
	vel.angular.z = 0

	# Rotate in Place at 90 Degrees Per second.
	rotate_vel = Twist()
        rotate_vel.linear.x = 0
	rotate_vel.angular.z = 1.57075

	# Set update rate at 10 Hz
	rate = rospy.Rate(10);

	# Run this loop forever.
	while True:
		# Move forward for 50 time ticks. (0.5 m/s * 10 seconds * 10 ticks per second)
		for i in range(50):
			self.set_velocity.publish(vel)
            		rate.sleep()
		rospy.loginfo("Moved 5 M...")
		# Rotate in place for 10 time ticks.
		for i in range(10):
			self.set_velocity.publish(rotate_vel)
    			rate.sleep()
		rospy.loginfo("Turned 90 Deg...")
                        
        
    def shutdown(self):

        rospy.loginfo("Stop Action")
	stop_vel = Twist()
        stop_vel.linear.x = 0
	stop_vel.angular.z = 0
        self.set_velocity.publish(stop_vel)

        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        turtlebot_move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")

