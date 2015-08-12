# NOTES:
# - currently the node dies with code -11, which (according to http://answers.ros.org/question/185197/process-has-died/) is a segmentation fault

cmake_minimum_required(VERSION 2.8.3)
project(pmd_nano)

SET(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive -Wno-deprecated -Wno-sign-compare")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
SET(CMAKE_CXX_LINK_FLAGS_DEBUG "-pg")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED COMPONENTS highgui imgproc)
# Do NOT use OpenCV 3.0.0 or all hell will break loose. Package incompatilibity will be a fascinating thing to deal with...

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
#  LIBRARIES pmd_nano
 CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs std_msgs uvc_camera
 DEPENDS
   system_lib # Boost PCL OpenCV
   boost_system
   PCL
   opencv_highgui opencv_imgproc
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(pmd_nano
#   src/${PROJECT_NAME}/pmd_nano_node.cpp
# )

# Declare executables and link
# The image and cloud publisher
add_executable(pmd_nano_publisher
    src/pmd_nano_publisher.cpp
    src/ColorCamera.cpp
    src/DepthCamera.cpp
    src/PMDNano.cpp
)
target_link_libraries(${PROJECT_NAME}_publisher ${catkin_LIBRARIES})
#link_directories(${PROJECT_NAME}_node ${Boost_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}_publisher boost_system)
link_directories(${PROJECT_NAME}_node ${PCL_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}_publisher opencv_highgui opencv_imgproc)
target_link_libraries(${PROJECT_NAME}_publisher pthread)
target_link_libraries(${PROJECT_NAME}_publisher ${PROJECT_SOURCE_DIR}/lib/libpmdaccess2.so)
# The image subscriber
add_executable(pmd_nano_image_subscriber
    src/pmd_nano_image_subscriber.cpp
)
target_link_libraries(${PROJECT_NAME}_image_subscriber ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_image_subscriber opencv_highgui opencv_imgproc)
# The cloud subscriber
add_executable(pmd_nano_cloud_subscriber
    src/pmd_nano_cloud_subscriber.cpp
)
target_link_libraries(${PROJECT_NAME}_cloud_subscriber boost_system)
link_directories(${PROJECT_NAME}_cloud_subscriber ${PCL_LIBRARY_DIRS})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pmd_nano.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
