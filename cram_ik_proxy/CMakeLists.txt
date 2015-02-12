cmake_minimum_required(VERSION 2.8.3)
project(cram_ik_proxy)

find_package(catkin REQUIRED COMPONENTS
  cram_language
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

