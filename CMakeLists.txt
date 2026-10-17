cmake_minimum_required(VERSION 3.0.2)
project(patrol)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  moma_service
  moveit_whole_body_ik
)

add_service_files(
  FILES
  SendArmPoint.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_runtime
  moma_service
  moveit_whole_body_ik
)

catkin_install_python(
  PROGRAMS
    scripts/arm_point_sender.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)