#!/usr/bin/env python
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy
import tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class Detector():

   """ Detector for Crazyflie -- identified by green ball mounted on battery in Kinect image """
   """ Crazyflie's location is published as a tf transform of its x, y, z with respect to  """
   """   the Kinect v2's ir coordinate frame """

   def __init__(self):

      # initialize ROS node and transform publisher
      rospy.init_node('crazyflie_detector', anonymous=True)
      self.pub_tf = tf.TransformBroadcaster()

      self.rate = rospy.Rate(50.0)                      # publish transform at 50 Hz

      # initialize values for crazyflie location on Kinect v2 image
      self.cf_u = 0                        # u is pixels left(0) to right(+)
      self.cf_v = 0                        # v is pixels top(0) to bottom(+)
      self.cf_d = 0                        # d is distance camera(0) to crazyflie(+) from depth image
      self.last_d = 0                      # last non-zero depth measurement

      # crazyflie orientation to Kinect v2 image (Euler)
      self.r = -1.5708
      self.p = 0
      self.y = -3.1415

      # Convert image from a ROS image message to a CV image
      self.bridge = CvBridge()

      cv2.namedWindow("KinectV2", 1)

      # Wait for the camera_info topic to become available
      rospy.wait_for_message('/kinect2/qhd/camera_info', CameraInfo)

      # Subscribe to Kinect v2 sd camera_info to get image frame height and width
      rospy.Subscriber('/kinect2/qhd/camera_info', CameraInfo, self.camera_data, queue_size=1)

      # Subscribe to registered color and depth images
      rospy.Subscriber('/kinect2/qhd/image_color_rect', Image, self.image_callback, queue_size=1)
      rospy.Subscriber('/kinect2/qhd/image_depth_rect', Image, self.depth_callback, queue_size=1)

      self.rate.sleep()                        # suspend until next cycle

   # This callback function sets parameters regarding the camera.
   def camera_data(self, data):
      # set values on the parameter server
      rospy.set_param('camera_link', data.header.frame_id)  # kinect2_ir_optical_frame
      rospy.set_param('camera_height', data.height)         # sd height is 424 / qhd height is 540
      rospy.set_param('camera_width', data.width)           # sd width is 512 / qhd width is 960

      # set values for local variables
      self.cam_height = data.height
      self.cam_width = data.width


   # This callback function handles processing Kinect color image, looking for the green ball
   #  on the Crazyflie.
   def image_callback(self, msg):

      # convert ROS image to OpenCV image
      try:
         image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
      except CvBridgeError as e:
         print(e)

      # create hsv image of scene
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

      # find green objects in the image
      lower_green = numpy.array([50, 50, 177], numpy.uint8)      # fluffy green ball
      upper_green = numpy.array([84, 150, 255], numpy.uint8)
      mask = cv2.inRange(hsv, lower_green, upper_green)

      # dilate and erode with kernel size 11x11
      cv2.morphologyEx(mask, cv2.MORPH_CLOSE, numpy.ones((11,11))) 

      # find all of the contours in the mask image
      contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      self.contourLength  = len(contours)

      # Check for at least one ball found
      if self.contourLength < 1:
         print "No objects found"
         sys.exit("No objects found")        # if no Crazyflie in image, exit process

      ## Loop through all of the contours, and get their areas
      area = [0.0]*len(contours)
      for i in range(self.contourLength):
         area[i] = cv2.contourArea(contours[i])

      #### Ball #### the largest "green" object
      ball_image = contours[area.index(max(area))]

      # Find the circumcircle of the green ball and draw a blue outline around it
      (self.cf_u,self.cf_v),radius = cv2.minEnclosingCircle(ball_image)
      ball_center = (int(self.cf_u),int(self.cf_v))
      ball_radius = int(radius)
      cv2.circle(image, ball_center, ball_radius, (255,0,0), 2)

      # show image with green ball outlined with a blue circle
      cv2.imshow ("KinectV2", image)
      cv2.waitKey(3)


   # This callback function handles processing Kinect depth image, looking for the depth value 
   #   at the location of the center of the green ball on top of Crazyflie.
   def depth_callback(self, msg):

      # create OpenCV depth image using defalut passthrough encoding
      try:
         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
      except CvBridgeError as e:
         print(e)

      # using green ball (u, v) position, find depth value of Crazyflie point and divide by 1000
      # to change millimeters into meters (for Kinect sensors only)
      self.cf_d = depth_image[self.cf_v, self.cf_u] / 1000.0
      rospy.loginfo("Depth: x at %d  y at %d  z at %f", int(self.cf_u), int(self.cf_v), self.cf_d)

      # if depth value is zero, use the last non-zero depth value
      if self.cf_d == 0:
         self.cf_d = self.last_d
      else:
         self.last_d = self.cf_d

      # publish Crazyflie tf transform
      self.update_cf_transform (self.cf_u, self.cf_v, self.cf_d)


   # This function builds the Crazyflie base_link tf transform and publishes it.
   def update_cf_transform(self, x, y, z):

      # send position as transform from the parent "kinect2_ir_optical_frame" to the
      # child "crazyflie/base_link" (described by crazyflie.urdf.xacro)
      self.pub_tf.sendTransform(( x,
                                  y,
                                  z),
                               tf.transformations.quaternion_from_euler(self.r, self.p, self.y),
                               rospy.Time.now(),
                               "crazyflie/base_link", "kinect2_ir_optical_frame")

      rospy.loginfo("Sent CF transform %f %f %f", x, y, z)


if __name__ == '__main__':

   # start up the detector node and run until shutdown by interrupt
   try:
      detector = Detector()
      rospy.spin()

   except rospy.ROSInterruptException:
      rospy.loginfo("Detector node terminated.")

   # close all terminal windows when process is shut down
   cv2.destroyAllWindows()


