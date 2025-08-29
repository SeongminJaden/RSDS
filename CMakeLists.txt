cmake_minimum_required(VERSION 3.5)
project(rsds)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/ggm_model.cpp
  src/legendre.cpp
  src/leo_gravity_plugin.cpp
)

# Compile shared library for Gazebo plugin
add_library(libleo_gravity_plugin SHARED ${SOURCES})
target_link_libraries(libleo_gravity_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(libleo_gravity_plugin
  rclcpp
  ignition-math6
)

# Install plugin library
install(TARGETS libleo_gravity_plugin
  DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install data (GGM05C)
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_package()
