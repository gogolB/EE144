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

        rate = rospy.Rate(10);
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

