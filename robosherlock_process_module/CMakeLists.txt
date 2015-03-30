cmake_minimum_required(VERSION 2.8.3)
project(robosherlock_process_module)

find_package(catkin REQUIRED COMPONENTS
  roslisp
  alexandria
  cram_plan_failures
  cram_process_modules
  cram_roslisp_common
  bullet_reasoning
  bullet_reasoning_designators
  cram_designators
  cram_plan_knowledge
  cram_manipulation_knowledge
  cl_transforms
  cram_projection
  cram_roslisp_common
  cl_semantic_map_utils
  semantic_map_costmap
  actionlib
  cram_designators
  cram_uima
  cram_moveit)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
