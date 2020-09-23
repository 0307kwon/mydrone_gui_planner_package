cmake_minimum_required(VERSION 2.8.3)
project(camera_to_world)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES camera_to_world
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(camera_to_world src/main_exe.cpp)
target_link_libraries(camera_to_world ${catkin_LIBRARIES})
