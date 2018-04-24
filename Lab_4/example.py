import rospy
from std_msgs.msg import String, Empty
from time import time
import geometry_msgs.msg
import tf
#this example only shows how to get robot pose
#set up node
rospy.init_node('example', anonymous=False)
#reset odometry
reset_odom = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size = 10)
#this message take a few iterations to get through
timer=time()
while time()-timer<1.0:
      reset_odom.publish(Empty())
#Initialize the tf listener
tfListener = tf.TransformListener()
def get_pose():
  test_done = False
  while test_done == False:
    try:
	#The base frame is /base_footprint for the Turtlebot. The odometry frame is usually just /odom. We want to get the transform between /base_footprint frame and /odom frame. Providing rospy.Time(0) gives the latest available tranform. 
    	#position[0] is x and position[1] is y.
	(position, quaternion) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
	#change quaternion to Euler angles since we have not learned quaternion in class. Orientation is in the form of [roll, pitch, yaw]. 
      	orientation = tf.transformations.euler_from_quaternion(quaternion)
      	test_done = True
	#if you do not understand how the postion and orientation look like, you can add this sample to your lab 2 code, and observe the format of position and orientation.
      	rospy.loginfo("position: "+str(position))
      	rospy.loginfo("orientation: "+str(orientation))
    except:
      continue


if __name__ == '__main__':
  try:
    get_pose()
  except rospy.ROSInterruptException:
    pass

