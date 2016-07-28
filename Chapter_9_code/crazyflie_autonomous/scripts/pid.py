import rospy                     # use Time 

class PID (object):

   """This class treats controllers created to have PID control on a system."""

   # initialization of flight PID controller
   def __init__(self, kp, kd, ki, minOutput, maxOutput, integratorMin, integratorMax):

      self.m_kp = kp                       # weight of proportional control
      self.m_kd = kd                       # weight of derivative control
      self.m_ki = ki                       # weight of integral control
      self.m_minOutput = minOutput         # largest output from controller
      self.m_maxOutput = maxOutput         # smallest output from controller
      self.m_integratorMin = integratorMin    # maximum integral value
      self.m_integratorMax = integratorMax    # minimum integral value

      # initial variable values
      self.m_integral = 0
      self.m_previousError = 0
      self.m_previousTime = float(rospy.Time.to_sec(rospy.Time.now()))

   # reset of essential PID parameters
   def reset (self):
      self.m_integral = 0
      self.m_previousError = 0
      self.m_previousTime = float(rospy.Time.to_sec(rospy.Time.now()))

   # sets (or resets) the integral value of the PID controller 
   def setIntegral (self, integral):
      self.m_integral = integral

   # returns the value of ki
   def set_ki (self):
      return self.m_ki

   # applies proportional, derivative, and integral control to determine optimal control value
   def update (self, value, targetValue):

      # record current value and target value and find current time
      rospy.loginfo("values are %f and %f", value, targetValue)
      time = float(rospy.Time.to_sec(rospy.Time.now()))

      # delta time is the time since the last update
      dt = time - self.m_previousTime

      # error is the difference between the current value and the target value
      error = targetValue - value

      # determine the integral value based on the delta time and current error
      self.m_integral += error * dt

      # assure that integral is within the max and min otherwise use the central value (max or min)
      self.m_integral = max(min(self.m_integral, self.m_integratorMax), self.m_integratorMin)
      rospy.loginfo("dt is %f, error is %f and m_integral is %f", dt, error, self.m_integral)

      # calculate proportional based on kp and current error
      p = self.m_kp * error

      # calculate derivative based on kd and the differenced in the current error and the last error
      # and divide by delta time (check is made that delta time is not zero to prevent divide by 0)
      d = 0
      if dt > 0:
         d = self.m_kd * (error - self.m_previousError) / dt

      # calculate integral based on ki and integral value of PID controller
      i = self.m_ki * self.m_integral

      # sum the proportional, derivative and integral to obtain controller output value
      output = p + d + i
      rospy.loginfo("P is %f, D is %f, I is %f", p, d, i) 

      # save current error and time for the next cycle
      self.m_previousError = error
      self.m_previousTime = time

      # assure that output value is within the max and min otherwise use the central value (max or min)
      return max(min(output, self.m_maxOutput), self.m_minOutput)

