cmake_minimum_required(VERSION 2.8.3)
project(cram_fingertips)

find_package(catkin REQUIRED COMPONENTS
  pr2_msgs
  roslisp
  roslisp_utilities
  cram_language
  cram_reasoning
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

