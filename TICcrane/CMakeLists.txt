cmake_minimum_required(VERSION 3.0.2)
project(tic_crane)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(FILES setTargetPose.srv)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES MoveToPosition.action
        Measurement.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stepper_motor
 CATKIN_DEPENDS roscpp std_msgs actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


find_library(PololuTIC_LIBRARY pololu-tic-1 HINTS $ENV{HOME}/usr/tic/lib )   
set(PololuTIC_INCLUDE_DIRS $ENV{HOME}/usr/tic/include/libpololu-tic-1)

include_directories(
  ${PololuTIC_INCLUDE_DIRS}
)

add_library(server src/server.cpp )
target_link_libraries(server ${catkin_LIBRARIES} ${PololuTIC_LIBRARY} )

add_executable(server_node src/server_node.cpp)
target_link_libraries(server_node server)


# ---Another valid method without shared library, left for educational purpouse---

# find_library(PololuTIC_LIBRARY pololu-tic-1 HINTS $ENV{HOME}/usr/tic/lib )   
# set(PololuTIC_INCLUDE_DIRS $ENV{HOME}/usr/tic/include/libpololu-tic-1)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${PololuTIC_INCLUDE_DIRS}
# )

# add_executable(server_node src/server_node.cpp src/server.cpp)
# target_link_libraries(server_node ${catkin_LIBRARIES} ${PololuTIC_LIBRARY})

catkin_install_python(PROGRAMS
scripts/state_machine.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
