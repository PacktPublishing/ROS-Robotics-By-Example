#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class Detector():

   """ Detector for the target -- identified by pink marker """
   """ Target's location is published as a PoseStamped message with its x, y, z with respect to  """
   """   the Kinect v2's image """

   def __init__(self):

      # initialize ROS node
      rospy.init_node('target_detector', anonymous=True)

      # initialize publisher for target pose, PoseStamped message, and set initial sequence number
      self.pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
      self.pub_pose = PoseStamped()
      self.pub_pose.header.seq = 0

      self.rate = rospy.Rate(1.0)                      # publish message at 1 Hz

      # initialize values for locating target on Kinect v2 image
      self.target_u = 0                        # u is pixels left(0) to right(+)
      self.target_v = 0                        # v is pixels top(0) to bottom(+)
      self.target_d = 0                        # d is distance camera(0) to target(+) from depth image
      self.target_found = False                # flag initialized to False
      self.last_d = 0                          # last non-zero depth measurement

      # target orientation to Kinect v2 image (Euler)
      self.r = 0
      self.p = 0
      self.y = 0

      # Convert image from a ROS image message to a CV image
      self.bridge = CvBridge()

      # Wait for the camera_info topic to become available
      rospy.wait_for_message('/kinect2/qhd/image_color_rect', Image)

      # Subscribe to registered color and depth images
      rospy.Subscriber('/kinect2/qhd/image_color_rect', Image, self.image_callback, queue_size=1)
      rospy.Subscriber('/kinect2/qhd/image_depth_rect', Image, self.depth_callback, queue_size=1)

      self.rate.sleep()                        # suspend until next cycle

   # This callback function handles processing Kinect color image, looking for the pink target.
   def image_callback(self, msg):

      # convert ROS image to OpenCV image
      try:
         image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
      except CvBridgeError as e:
         print(e)

      # create hsv image of scene
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

      # find pink objects in the image
      lower_pink = numpy.array([139, 0, 240], numpy.uint8)
      upper_pink = numpy.array([159, 121, 255], numpy.uint8)
      mask = cv2.inRange(hsv, lower_pink, upper_pink)

      # dilate and erode with kernel size 11x11
      cv2.morphologyEx(mask, cv2.MORPH_CLOSE, numpy.ones((11,11))) 

      # find all of the contours in the mask image
      contours, heirarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      self.contourLength  = len(contours)

      # Check for at least one target found
      if self.contourLength < 1:
         print "No target found"

      else:                       # target found

         ## Loop through all of the contours, and get their areas
         area = [0.0]*len(contours)
         for i in range(self.contourLength):
            area[i] = cv2.contourArea(contours[i])

         #### Target #### the largest "pink" object
         target_image = contours[area.index(max(area))]

         # Using moments find the center of the object and draw a red outline around the object
         target_m = cv2.moments(target_image)
         self.target_u = int(target_m['m10']/target_m['m00'])
         self.target_v = int(target_m['m01']/target_m['m00'])
         points = cv2.minAreaRect(target_image)
         box = cv2.cv.BoxPoints(points)
         box = numpy.int0(box)
         cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
         rospy.loginfo("Center of target is x at %d and y at %d", int(self.target_u), int(self.target_v))

         self.target_found = True               # set flag for depth_callback processing

         # show image with target outlined with a red rectangle
         cv2.imshow ("Target", image)
         cv2.waitKey(3)

   # This callback function handles processing Kinect depth image, looking for the depth value 
   #   at the location of the center of the pink target.
   def depth_callback(self, msg):

      # process only if target is found
      if self.target_found == True:

         # create OpenCV depth image using default passthrough encoding
         try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
         except CvBridgeError as e:
            print(e)

         # using target (v, u) location, find depth value of point and divide by 1000
         # to change millimeters into meters (for Kinect sensors only)
         self.target_d = depth_image[self.target_v, self.target_u] / 1000.0

         # if depth value is zero, use the last non-zero depth value
         if self.target_d == 0:
            self.target_d = self.last_d
         else:
            self.last_d = self.target_d

         # record target location and publish target pose message
         rospy.loginfo("Target depth: x at %d  y at %d  z at %f", int(self.target_u), 
                             int(self.target_v), self.target_d)
         self.update_target_pose (self.target_u, self.target_v, self.target_d)

   # This function builds the target PoseStamped message and publishes it.
   def update_target_pose(self, x, y, z):

      # set position values for target location
      self.pub_pose.pose.position.x = x      # pose in camera pixels u
      self.pub_pose.pose.position.y = y      # pose in camera pixels v
      self.pub_pose.pose.position.z = z      # pose in meters from camera

      # determine quaternion values from euler angles of (0,0,0)
      quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
      self.pub_pose.pose.orientation.x = quaternion[0]
      self.pub_pose.pose.orientation.y = quaternion[1]
      self.pub_pose.pose.orientation.z = quaternion[2]
      self.pub_pose.pose.orientation.w = quaternion[3]

      # complete header information
      self.pub_pose.header.seq += 1
      self.pub_pose.header.stamp = rospy.Time.now()
      self.pub_pose.header.frame_id = "target"

      # publish pose of target
      self.pub.publish(self.pub_pose)

if __name__ == '__main__':

   # start up the detector node and run until shutdown by interrupt
   try:
      detector = Detector()
      rospy.spin()

   except rospy.ROSInterruptException:
      rospy.loginfo("Detector node terminated.")

   # close all terminal windows when process is shut down
   cv2.destroyAllWindows()


