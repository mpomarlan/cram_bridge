cmake_minimum_required(VERSION 2.8.3)
project(cram_fccl)

find_package(catkin REQUIRED COMPONENTS
  cram_language
  roslisp
  actionlib_lisp
  cl_transforms
  cl_feature_constraints
  fccl_msgs
  geometry_msgs
  std_msgs
  lisp_unit
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

