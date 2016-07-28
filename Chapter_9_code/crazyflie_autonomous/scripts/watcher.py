#!/usr/bin/env python

import rospy
import tf
from std_srvs.srv import Empty

""" Watcher for Crazyflie -- monitors the location of Crazyflie in the Kinect image     """
"""  and will send a service call to the land service if Crazyflie's location is within """
"""  the outer boundary of the image """

if __name__ == '__main__':

   # initialize ROS node and tf frame info from parameter server, default parameters given
   rospy.init_node('window_watcher', anonymous=True)
   crazyflie_frame = rospy.get_param("~frame", "crazyflie/base_link")
   camera_frame = rospy.get_param("~cam_frame", "kinect2_ir_optical_frame")
   r = rospy.get_param("~rate", 10)     # default rate is 10 Hz

   # subscribe to crazyflie tf
   listener = tf.TransformListener()

   # get camera info from parameter server
   camera_height = rospy.get_param('camera_height')      # sd height is 424; qhd height is 540
   camera_width = rospy.get_param('camera_width')        # sd width is 512; qhd width is 960

   # continue to process at the given rate until a shutdown request is received
   rate = rospy.Rate(r)
   while not rospy.is_shutdown():

      # get the current transform between the camera_frame and the crazyflie_frame
      if listener.frameExists(camera_frame) and listener.frameExists(crazyflie_frame):
         t = listener.getLatestCommonTime(camera_frame, crazyflie_frame)
         trans, rotate = listener.lookupTransform(camera_frame, crazyflie_frame, t)
         rospy.loginfo("watcher: cf_trans %f %f %f", trans[0], trans[1], trans[2])

         # check Crazyflie's x and y location with respect to the Kinect image --
         # if Crazyflie is within 100 pixels of the image's horizontal edges (x)
         # or if it is with in 20 pixels of the image's vertical edges (y)
         if (trans[0] < 100) or (trans[0] > (camera_width - 100)) or \
             (trans[1] < 20) or (trans[1] > (camera_height - 20)):

            # wait until land service is available, then create a handle for it
            rospy.loginfo("Crazyflie outside of window %f %f %f", trans[0], trans[1], trans[2])
            rospy.loginfo("Landing requested")
            rospy.wait_for_service('/crazyflie/land')

            # request the land service
            try:
               _land = rospy.ServiceProxy('/crazyflie/land', Empty)
               _land()
            except rospy.ServiceException, e:
               rospy.loginfo("Service call failed: %s", e)

      # suspend until next cycle
      rate.sleep()
