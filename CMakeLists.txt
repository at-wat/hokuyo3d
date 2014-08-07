cmake_minimum_required(VERSION 2.8.3)
project(hokuyo3d)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(Boost 1.53 REQUIRED system serialization)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(hokuyo3d src/hokuyo3d.cpp src/vssp.hpp)
target_link_libraries(hokuyo3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})
