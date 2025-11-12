cmake_minimum_required(VERSION 3.8)  # 3.8 is the common ROS 2 baseline
project(interfaces_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Only keep these if your .srv/.msg reference them
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(builtin_interfaces REQUIRED)  # uncomment if you use builtin types (e.g., time, duration)

# List all your interface files here
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InverseKinematics.srv"
  DEPENDENCIES geometry_msgs std_msgs  # add builtin_interfaces if you use it
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

