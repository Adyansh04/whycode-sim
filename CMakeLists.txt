cmake_minimum_required(VERSION 3.0.2)
project(fiducial_gazebo_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fiducial_gazebo_sim
#  CATKIN_DEPENDS roscpp gazebo_ros
#  DEPENDS system_lib
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/..
)


