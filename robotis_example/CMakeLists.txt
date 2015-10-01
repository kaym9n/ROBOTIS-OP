cmake_minimum_required(VERSION 2.8.3)
project(robotis_example)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_controller
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#   std_msgs
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
  LIBRARIES robotis_example
  CATKIN_DEPENDS dynamixel_controller roscpp std_msgs
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
# add_library(robotis_example
#   src/${PROJECT_NAME}/robotis_example.cpp
# )

## Declare a cpp executable
add_executable(example_read_write 
  src/read_write.cpp
)

# add_dependencies(robotis_example_node robotis_example_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(example_read_write
  dynamixel_controller
  ${catkin_LIBRARIES}
)

