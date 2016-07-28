#!/usr/bin/env python

import rospy
import tf
from pid import PID
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty

class CF_Controller():

   """Controller to fly a Crazyflie in a space with Kinect v2 position feedback"""
   """Adapted to Python from Wolfgang Hoenig's controller.cpp in package crazyflie_controller"""

   def __init__(self):

      self.ff = 0.45                                   # adjustment for takeoff thrust
      self.thrust = 0.0                                # initialize thrust to 0
      self.target = False                              # initialize target sighting to False

      # tuple of Crazyflie flight states
      self.flight_state = ['idle', 'takeoff', 'hover', 'flight', 'land']    
      self._cf_state = 'idle'                          # initial flight state
      self.takeoff_position = [0.0, 0.0, 0.0]          # inital takeoff and hover positions
      self.hover_position = [0.0, 0.0, 0.0]
      self.last_depth = 0.0                # variable to keep non-zero value for camera depth 

      # initialize services for land and takeoff
      # emergency is handled by crazyflie_server.cpp in crazyflie_ros/crazyflie_driver package
      s1 = rospy.Service("/crazyflie/land", Empty, self._Land)
      s2 = rospy.Service("/crazyflie/takeoff", Empty, self._Takeoff)

      # subscribe to target pose
      self.target_position = PoseStamped()
      rospy.Subscriber('target_pose', PoseStamped, self._update_target_pose, queue_size=1)

      # Initialize the tf listener
      self.listener = tf.TransformListener()

      # initialize publisher for crazyflie command velocity (geometry_Twist)
      self.fly = Twist()                           # set the fly command
      self.velocity_pub = rospy.Publisher ('/crazyflie/cmd_vel', Twist, queue_size=1)

      # get camera info from parameter server
      self.camera_height = rospy.get_param('camera_height')      # sd height is 424; qhd height is 540
      self.camera_width = rospy.get_param('camera_width')        # sd width is 512; qhd width is 960

      # create flight PID controllers in X, Y and Z 
      self.m_pidX = PID(rospy.get_param("~PIDs/X/kp"),
                        rospy.get_param("~PIDs/X/kd"),
                        rospy.get_param("~PIDs/X/ki"),
                        rospy.get_param("~PIDs/X/minOutput"),
                        rospy.get_param("~PIDs/X/maxOutput"),
                        rospy.get_param("~PIDs/X/integratorMin"),
                        rospy.get_param("~PIDs/X/integratorMax"))
      self.m_pidY = PID(rospy.get_param("~PIDs/Y/kp"),
                        rospy.get_param("~PIDs/Y/kd"),
                        rospy.get_param("~PIDs/Y/ki"),
                        rospy.get_param("~PIDs/Y/minOutput"),
                        rospy.get_param("~PIDs/Y/maxOutput"),
                        rospy.get_param("~PIDs/Y/integratorMin"),
                        rospy.get_param("~PIDs/Y/integratorMax"))
      self.m_pidZ = PID(rospy.get_param("~PIDs/Z/kp"),
                        rospy.get_param("~PIDs/Z/kd"),
                        rospy.get_param("~PIDs/Z/ki"),
                        rospy.get_param("~PIDs/Z/minOutput"),
                        rospy.get_param("~PIDs/Z/maxOutput"),
                        rospy.get_param("~PIDs/Z/integratorMin"),
                        rospy.get_param("~PIDs/Z/integratorMax"))
      self.m_pidYaw = PID(rospy.get_param("~PIDs/Yaw/kp"),
                          rospy.get_param("~PIDs/Yaw/kd"),
                          rospy.get_param("~PIDs/Yaw/ki"),
                          rospy.get_param("~PIDs/Yaw/minOutput"),
                          rospy.get_param("~PIDs/Yaw/maxOutput"),
                          rospy.get_param("~PIDs/Yaw/integratorMin"),
                          rospy.get_param("~PIDs/Yaw/integratorMax"))


   # This service processes the Takeoff request.
   def _Takeoff(self, req):
      rospy.loginfo("Takeoff requested!")
      self._cf_state = 'takeoff'
      return ()
 
   # This service processes the Land request.
   def _Land(self, req):
      rospy.loginfo("Landing requested!")
      self._cf_state = 'land'
      return ()


   # This callback function puts the target PoseStamped message into a local variable.
   def _update_target_pose(self, msg):
      self.target_position = msg
      self.target = True

   # This function gets the current transform between the target_frame and the source_frame.
   def _getTransform(self, target_frame, source_frame):

      if self.listener.frameExists(target_frame) and self.listener.frameExists(source_frame):
         t = self.listener.getLatestCommonTime(target_frame, source_frame)
         result, other = self.listener.lookupTransform(target_frame, source_frame, t)
         return result, other

   # This function calls the reset function for all the PID controllers.
   def _pidReset(self):
      self.m_pidX.reset()
      self.m_pidY.reset()
      self.m_pidZ.reset()
      self.m_pidYaw.reset()


   # This process handles all of the flight command messages. 
   def iteration(self, event):

      try:
         # delta time is in a fraction of a second (0.02 sec for 50 Hz)
         dt = float(rospy.Time.to_sec(event.current_real)) - float(rospy.Time.to_sec(event.last_real))

      except (AttributeError, TypeError):
         dt = 0
      print "dt is %f" % dt

      # receive tf transform on the location of Crazyflie; log critical values of x, y, z
      (cf_trans, cf_rot) = self._getTransform("kinect2_ir_optical_frame", cf_frame) 
      rospy.loginfo("cf_trans %f %f %f", cf_trans[0], cf_trans[1], cf_trans[2])


      ##### Idle ########
      if self._cf_state == 'idle':
         # set command velocity message values to zero; thrust value to zero
         self.fly.linear.x = 0.0
         self.fly.linear.y = 0.0
         self.fly.linear.z = 0.0
         self.thrust = 0.0

         # save current position as the takeoff position
         self.takeoff_position = cf_trans


      ##### Hover ########
      # use x, y, and z PID controllers to keep Crazyflie's position the same as the hover position
      #
      elif self._cf_state == 'hover':

         # use camera -x position
         # calculate PID control value for Crazyflie x control
         self.fly.linear.x = self.m_pidX.update((self.camera_width - cf_trans[0]), 
                                                (self.camera_width - self.hover_position[0]))

         # use camera -z position; make sure depth value is not 0; if it is 0, use previous value
         # calculate PID control value for Crazyflie y control
         if cf_trans[2] == 0.0:
            rospy.loginfo("Camera z is zero")
            self.fly.linear.y = self.m_pidY.update(self.hover_position[2], self.last_depth)
         else:
            self.fly.linear.y = self.m_pidY.update(self.hover_position[2], cf_trans[2])
            self.last_depth = cf_trans[2]

         # use camera -y position
         # calculate PID control value for Crazyflie z control
         self.fly.linear.z = self.m_pidZ.update((self.camera_height - cf_trans[1]), 
                                                (self.camera_height - self.hover_position[1]))   

         # record values to log file
         rospy.loginfo("hover_position %f %f %f", self.hover_position[0], self.hover_position[1],
            self.hover_position[2])
         rospy.loginfo("Velocity msg published x %f y %f z % f", self.fly.linear.x, self.fly.linear.y,
            self.fly.linear.z)


      ##### Takeoff ########
      # increase the thrust (z) value in the command velocity message until takeoff is achieved
      #
      elif self._cf_state == 'takeoff':

         self.fly.linear.x = 0.0             # set cmd_vel x and y to 0
         self.fly.linear.y = 0.0

         rospy.loginfo("new upper limit %f", self.takeoff_position[1]-25)

         # increase thrust until position is 25 pixels above the takeoff position
         #   (in camera -y direction) 
         if (cf_trans[1] < (self.takeoff_position[1]-25)) or (self.thrust > 50000):

            # when 25 pixels above the takeoff position is achieved,
            # reset controllers; log values and achievement; change state to flight
            self._pidReset()
            rospy.loginfo("Takeoff thrust %f, ki %f", self.thrust, self.m_pidZ.set_ki())
            self.m_pidZ.setIntegral((self.thrust - 1500.0)/ self.m_pidZ.set_ki())
            rospy.loginfo("Takeoff achieved!")
            self._cf_state = 'flight'
            self.thrust = 0.0

         # if position has not been achieved; increase thrust (z) value in command velocity;
         #  calculation is based on delta time (time elapsed) and fudge factor;
         #  slow but steady increments that decrease above 36000
         else:
            if self.thrust < 36000:
               self.thrust += 10000 * dt * self.ff
            else:
               self.thrust += 3000 * dt * self.ff

            self.fly.linear.z = self.thrust
            rospy.loginfo("Takeoff Thrust value %f", self.fly.linear.z)

         # print thrust values and command velocity values to screen for debugging
         print "Thrust is %f" % self.thrust
         print "Velocity msg published x %f y %f z % f" % (self.fly.linear.x, self.fly.linear.y,
            self.fly.linear.z)

      ##### Land ########
      elif self._cf_state == 'land':

         # reduce pitch and roll to zero maintain thrust at 30000 for Crazyflie landing
         self.fly.linear.x = 0.0
         self.fly.linear.y = 0.0
         self.fly.linear.z = 30000.0
         self.thrust = 30000.0

         # log achievement and change state to idle
         rospy.loginfo("Landing achieved!")
         self._cf_state = 'idle'


      ##### Flight ########
      # use x, y, and z PID controllers to move Crazyflie's position to the target position
      #
      elif self._cf_state == 'flight':

         # check for target pose, if none then hover
         if not self.target:
            rospy.loginfo("Hover position set!!!!!!!!!!")
            self.hover_position = cf_trans
            rospy.loginfo("Hover pose is %f %f %f", self.hover_position[0], self.hover_position[1], 
                                                    self.hover_position[2])
            self._cf_state = 'hover'

         # if target pose exists, head for target
         else:
            rospy.loginfo("Flying to target!!!")
            rospy.loginfo("Target position is x %f, y %f, z %f", self.target_position.pose.position.x,
                        self.target_position.pose.position.y, self.target_position.pose.position.z)
            
            # use camera -x position
            # calculate PID control value for Crazyflie x control
            self.fly.linear.x = self.m_pidX.update((self.camera_width - cf_trans[0]), 
                                (self.camera_width - self.target_position.pose.position.x))  
       
            # use camera -z position
            # calculate PID control value for Crazyflie y control
            self.fly.linear.y = self.m_pidY.update(cf_trans[2], self.target_position.pose.position.z)

            # use camera -y position
            # calculate PID control value for Crazyflie z control
            self.fly.linear.z = self.m_pidZ.update((self.camera_height - cf_trans[1]), 
                                (self.camera_height - self.target_position.pose.position.y + 25))   
#  for testing only                                              (self.camera_height - cf_trans[1]))   

         # record values to log file
         rospy.loginfo("Target_position %f %f %f", self.target_position.pose.position.x,
                 self.target_position.pose.position.y, self.target_position.pose.position.z)
         rospy.loginfo("Velocity msg published x %f y %f z % f", self.fly.linear.x, self.fly.linear.y,
                 self.fly.linear.z)

      # publish command velocity message to Crazyflie
      self.velocity_pub.publish(self.fly)

      # log flight state
      rospy.loginfo("CF_state is %s", str(self._cf_state))


if __name__ == '__main__':

   # initialize ROS node
   rospy.init_node('crazyflie_controller', anonymous=True)

   # set parameters on parameter server
   cf_frame = rospy.get_param("~frame", "crazyflie/base_link")
   frequency = rospy.get_param("frequency", 50.0)

   # start up the controller node and run until shutdown by interrupt
   try:
      controller = CF_Controller()
      rospy.Timer(rospy.Duration(1.0/frequency), controller.iteration)   

      rospy.spin()                  # keep process alive

   except rospy.ROSInterruptException:
      rospy.loginfo("CF_controller node terminated.")

