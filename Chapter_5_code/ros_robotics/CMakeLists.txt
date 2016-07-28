cmake_minimum_required(VERSION 2.8.3)
project(ros_robotics)

find_package(catkin REQUIRED)
catkin_package()

## Install ##

foreach(dir config launch meshes urdf worlds)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(
  FILES urdf.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

