<launch>
   <!-- values passed by command line input -->     
   <arg name="model" />
   <arg name="gui" default="False" />

   <!-- set these parameters on Parameter Server -->
   <param name="robot_description" textfile="$(find ros_robotics)/urdf/$(arg model)" />
   <param name="use_gui" value="$(arg gui)"/>

   <!-- Start 3 nodes: joint_state_publisher, robot_state_publisher and rviz -->
   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

   <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find ros_robotics)/urdf.rviz" required="true" />
   <!-- (required = "true") if rviz dies, entire roslaunch will be killed -->
</launch>
