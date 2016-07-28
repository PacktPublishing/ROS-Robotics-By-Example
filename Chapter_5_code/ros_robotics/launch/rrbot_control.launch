<launch>

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find ros_robotics)/config/rrbot_control.yaml" command="load"/>

  <!-- load the controllers -->
  <node name="control_spawner" pkg="controller_manager" type="spawner" respawn="false"
	output="screen" ns="/rrbot" args="joint_state_controller
					  joint_base_mid_position_controller
					  joint_mid_top_position_controller
                                          left_gripper_joint_position_controller
                                          right_gripper_joint_position_controller"/>

  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
	respawn="false" output="screen">
    <remap from="/joint_states" to="/rrbot/joint_states" />
  </node>

</launch>
