cmake_minimum_required(VERSION 2.8.3)
project(fetch_fridge)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
  PoseStampedBooled.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
