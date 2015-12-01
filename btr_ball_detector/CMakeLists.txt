cmake_minimum_required(VERSION 2.8.3)
project(btr_ball_detector)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
      roscpp rospy std_msgs sensor_msgs geometry_msgs image_transport cv_bridge message_generation dynamic_reconfigure)

## System dependencies are found with CMake's conventions
#find_package(ball_detector REQUIRED)

#SRCS files
SET(SRCS 
      src/ball_detector_node.cpp
)

SET(ball_detector
    /home/'username'/ball_detector
)

SET(ball_detector_INCLUDE_DIR
    ${ball_detector}/src
)

SET(ball_detector_LIB
    ${ball_detector}/lib/libball_detector.so
)

## Add messages to be generated
add_message_files(FILES circleSetStamped.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/detectorParams.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS roscpp rospy geometry_msgs image_transport cv_bridge message_runtime dynamic_reconfigure
      DEPENDS ball_detector
)

## Specify additional locations of header files
include_directories(${ball_detector_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(btr_ball_detector_node ${SRCS} src/ball_detector_main.cpp)
add_dependencies(btr_ball_detector_node ${PROJECT_NAME}_EXPORTED_TARGETS)
target_link_libraries(btr_ball_detector_node ${ball_detector_LIB} ${catkin_LIBRARIES})
