cmake_minimum_required(VERSION 3.5)
project(laser_proc)

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs rosconsole nodelet pluginlib)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(class_loader REQUIRED)

find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIRS})

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_proc_library laser_publisher laser_transport laser_proc_ROS LaserProcNodelet
#  CATKIN_DEPENDS roscpp sensor_msgs rosconsole nodelet
#  DEPENDS
#)

set(INCLUDE_DIRS
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(LIBS
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${class_loader_LIBRARIES}
  ${Boost_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${INCLUDE_DIRS})

## Library that does actual conversion
add_library(laser_proc_library src/LaserProc.cpp)
target_link_libraries(laser_proc_library ${LIBS})

## Publisher library
add_library(laser_publisher src/LaserPublisher.cpp)
target_link_libraries(laser_publisher laser_proc_library ${LIBS})

## Factory for making LaserPublishers
add_library(laser_transport src/LaserTransport.cpp)
target_link_libraries(laser_transport laser_publisher ${LIBS})

## ROS Library
add_library(laser_proc_ROS src/LaserProcROS.cpp)
target_link_libraries(laser_proc_ROS laser_transport ${LIBS})

## Nodelet wrapper
add_library(LaserProcNodelet src/LaserProcNodelet.cpp)
target_link_libraries(LaserProcNodelet laser_proc_ROS ${LIBS})

## Node wrapper
add_executable(laser_proc src/laser_proc.cpp)
target_link_libraries(laser_proc laser_proc_ROS ${LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS laser_proc_library laser_publisher laser_transport laser_proc_ROS LaserProcNodelet laser_proc
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelets.xml
        DESTINATION share
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
#if(BUILD_TESTING)
#  find_package(ament_cmake_nose REQUIRED)
#  ament_add_nose_test(laser_proc_nose_tests test)
#endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(laser_proc_library laser_publisher laser_transport laser_proc_ROS LaserProcNodelet ${LIBS})

ament_package()
