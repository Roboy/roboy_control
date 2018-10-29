cmake_minimum_required(VERSION 2.8.3)
project(xylophone_plugin)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
        common_utilities
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
message(STATUS ${GAZEBO_INCLUDE_DIRS})

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
  DEPENDS common_utilities
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(HitDetection  src/HitDetectionPlugin.cpp)
target_link_libraries(HitDetection ${GAZEBO_libraries}  ${roscpp_LIBRARIES} ${catkin_LIBRARIES})