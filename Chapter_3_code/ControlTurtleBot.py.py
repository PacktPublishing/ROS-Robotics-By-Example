#!/usr/bin/env python
# Execute as a python script  
# Set linear and angular values of TurtleBot's speed and turning. 
import rospy				# Needed to create a ROS node
from geometry_msgs.msg import Twist     # Message that moves base

class ControlTurtleBot():
    def __init__(self):
        # ControlTurtleBot is the name of the node sent to the master
        rospy.init_node('ControlTurtleBot', anonymous=False)

	# Message to screen
	rospy.loginfo(" Press CTRL+c to stop TurtleBot")

        # Keys CNTL + c will stop script   
        rospy.on_shutdown(self.shutdown)
        
	#Publisher will send Twist message on topic cmd_vel_mux/input/navi
       
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
	# TurtleBot will receive the message 10 times per second. 
        rate = rospy.Rate(10);   
	# 10 Hz is fine as long as the processing does not exceed 1/10 second.

        # Twist is a type of geometry_msgs for linear and angular velocity
        move_cmd = Twist()
	# Linear speed in x in meters/second is + (forward) or - (backwards)
        move_cmd.linear.x = 0.3	# Modify this value to change speed
	# let's turn at 0 radians/s
	move_cmd.angular.z = 0	# Modify this value to cause rotation rad/s

	# Loop and TurtleBot will move until you type CNTL+c
        while not rospy.is_shutdown():
	    # publish the Twist values to the TurtleBot node /cmd_vel_mux
            self.cmd_vel.publish(move_cmd)
	    # wait for 0.1 seconds (10 HZ) and publish again
            rate.sleep()
                        
        
    def shutdown(self):
        # You can stop turtlebot by publishing an empty Twist message 
        rospy.loginfo("Stopping TurtleBot")
	# 
        self.cmd_vel.publish(Twist())
	# Give TurtleBot time to stop
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        ControlTurtleBot()
    except:
        rospy.loginfo("End of the trip for TurtleBot")

