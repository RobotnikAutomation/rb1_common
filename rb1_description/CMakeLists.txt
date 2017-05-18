cmake_minimum_required(VERSION 2.8.3)
project(rb1_description)


find_package(catkin REQUIRED COMPONENTS
  message_runtime
  rb1_base_description
  rb1_torso_description
  kinova_description
  kinova_driver
  std_msgs
  std_srvs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

