#!/usr/bin/env python   # home_arms_zero.py 
#   
"""
Script to return Baxter's arms to a "home zero" position  
"""
# rospy - ROS Python API
import rospy
# baxter_interface - Baxter Python API
import baxter_interface
# initialize our ROS node, registering it with the Master
rospy.init_node('Home_Arms')
# create instances of baxter_interface's Limb class
limb_right = baxter_interface.Limb('right')
limb_left = baxter_interface.Limb('left')
# store the home position of the arms
home_zero_right = {'right_s0': 0.0, 'right_s1': 0.00, 'right_w0': 0.00, 'right_w1': 0.00, 'right_w2': 0.00, 'right_e0': 0.00, 'right_e1': 0.00}
home_zero_left = {'left_s0': 0.0, 'left_s1': 0.00, 'left_w0': 0.00, 'left_w1': 0.00, 'left_w2': 0.00, 'left_e0': 0.00, 'left_e1': 0.00}
# move both arms to home position
limb_right.move_to_joint_positions(home_zero_right)
limb_left.move_to_joint_positions(home_zero_left)
quit()


