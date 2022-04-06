cmake_minimum_required(VERSION 2.8.3)
project(body_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_service_files(
  FILES
  BodyPose.srv
)

add_message_files(
  FILES
  BodyInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_generation cv_bridge message_runtime
)
