cmake_minimum_required(VERSION 2.8.3)
project(cram_moveit)

find_package(catkin REQUIRED COMPONENTS moveit_msgs)

catkin_package(
  CATKIN_DEPENDS moveit_msgs
)
