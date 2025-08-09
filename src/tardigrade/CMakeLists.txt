cmake_minimum_required(VERSION 3.5)
project(tardigrade)

# Default to C++14 (if needed later)
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Build custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/SubThrusts.msg
  DEPENDENCIES std_msgs builtin_interfaces
)

# Install Python modules
ament_python_install_package(controls)
ament_python_install_package(vision)

# Install msg, urdf, and mesh files
install(DIRECTORY
  msg
  urdf
  meshes
  launch
  robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()