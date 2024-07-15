cmake_minimum_required(VERSION 3.0.2)
project(robot_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  std_msgs
  gazebo_ros
  geometry_msgs
  ackermann_msgs
  joint_state_publisher
  robot_state_publisher
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES robot_gazebo
 CATKIN_DEPENDS controller_manager roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_manager_node src/gazebo_manager_node.cpp)
target_link_libraries(gazebo_manager_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
