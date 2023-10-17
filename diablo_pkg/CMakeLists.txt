cmake_minimum_required(VERSION 3.0.2)
project(diablo_pkg)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MotionCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS message_runtime
)


add_library( diablo_pkg 
    src/Diffdrive_control.cpp 
)
target_link_libraries( diablo_pkg
    ${catkin_LIBRARIES} 
)  

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
