cmake_minimum_required(VERSION 2.8.3)
project(cram_beliefstate)

find_package(catkin REQUIRED COMPONENTS
  cram_language
  roslisp
  designator_integration_lisp)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS})
