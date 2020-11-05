cmake_minimum_required(VERSION 3.0.2)
project(hokuyo3d)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs roslint)
find_package(Boost 1.53 REQUIRED system serialization chrono thread)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

add_compile_options(-std=c++11)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

add_executable(hokuyo3d src/hokuyo3d.cpp)
target_link_libraries(hokuyo3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

install(TARGETS hokuyo3d
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
