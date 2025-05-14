cmake_minimum_required(VERSION 3.5)
project(car_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  std_msgs
)

# Include directories
include_directories(include)

# Add the AEBNode executable
add_executable(aeb_node src/aeb_node.cpp)
ament_target_dependencies(aeb_node ${dependencies})

# Add the BicycleControlNode executable
add_executable(bicycle_control_node src/bicycle_control_node.cpp)
ament_target_dependencies(bicycle_control_node ${dependencies})

# Add the AngleSimulationNode executable
add_executable(angle_simulation_node src/angle_simulation_node.cpp)
ament_target_dependencies(angle_simulation_node ${dependencies})

# Install the executables
install(TARGETS
  aeb_node
  bicycle_control_node
  angle_simulation_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()