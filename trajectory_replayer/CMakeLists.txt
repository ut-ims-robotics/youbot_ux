cmake_minimum_required(VERSION 3.0.2)
project(trajectory_replayer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  trajectory_recorder
)

add_service_files(
  FILES
  SendTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES trajectory_replayer
#  CATKIN_DEPENDS message_generation roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Action engine node
add_executable(trajectory_replayer_node
  src/trajectory_replayer_node.cpp
  src/trajectory_replayer.cpp
)

add_dependencies(trajectory_replayer_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(trajectory_replayer_node
  ${catkin_LIBRARIES}
)


