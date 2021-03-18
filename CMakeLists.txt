cmake_minimum_required(VERSION 3.5)
project(hokuyo3d)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system serialization chrono thread)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(Boost 1.53 REQUIRED system serialization chrono thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/hokuyo3d.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
