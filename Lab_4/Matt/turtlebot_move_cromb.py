#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist

class turtlebot_move():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)

        rospy.loginfo("Press CTRL + C to terminate")
 
        rospy.on_shutdown(self.shutdown)
        
        self.set_velocity = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        vel = Twist()
        vel.linear.x = 0.5
        vel.angular.z = 0
        
        vel_t = Twist()
        vel_t.linear.x = 0
        vel_t.angular.z = 1.57079632 #robot will rotate at 1.57rad /s for 2 sec
        
        #set update at 10 Hz
        rate = rospy.Rate(10);
        #allows the robot to move forward for 0.5m/s for 10 seconds
        #robot then turns 90 degrees
        #sampling rate of 10hZ (0.5m/s *10 seconds* 10 ticks per second = 50)
        #(0.5m/s *2 seconds* 10 ticks per second = 10)
        while (True):
            for i in range(50):
                self.set_velocity.publish(vel)
                rate.sleep()
            for i in range(10):
                self.set_velocity.publish(vel_t)
                rate.sleep()
                
        while not rospy.is_shutdown():
            self.set_velocity.publish(vel)
            rate.sleep()
        
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


        