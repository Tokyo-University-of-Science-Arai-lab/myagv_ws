cmake_minimum_required(VERSION 3.5)
project(my_param)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install parameters file
install(
  DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

ament_package()
