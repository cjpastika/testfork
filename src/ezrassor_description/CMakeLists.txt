cmake_minimum_required(VERSION 3.8)
project(ezrassor_description)

# Find ament_cmake - the ROS2 build system
find_package(ament_cmake REQUIRED)

# Install our directories to the share folder
# This makes them accessible to other packages via:
#   get_package_share_directory('ezrassor_description')
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# Register this as an ament package
ament_package()
