cmake_minimum_required(VERSION 3.5)
project(myagv_description)

# Find ament_cmake package
find_package(ament_cmake REQUIRED)

# Install URDF and mesh files
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package macro
ament_package()
