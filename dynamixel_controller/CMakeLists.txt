cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_controller)

find_package(catkin REQUIRED COMPONENTS
#  dynamixel_msgs
  roscpp
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   dynamixel_msgs#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dynamixel_controller
#  CATKIN_DEPENDS dynamixel_msgs roscpp sensor_msgs std_msgs
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(dynamixel_controller
  src/DynamixelController.cpp
  src/dynamixel/Dynamixel.cpp
  src/packet_control/SerialPort.cpp
  src/packet_control/GroupHandler.cpp
  src/packet_control/PacketHandler.cpp
  src/packet_control/PacketProtocol1.cpp
  src/packet_control/PacketProtocol2.cpp
)

## Declare a cpp executable
#add_executable(dynamixel_controller
#  src/main.cpp
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(dynamixel_controller dynamixel_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dynamixel_controller
  yaml-cpp
  ${catkin_LIBRARIES}
)
