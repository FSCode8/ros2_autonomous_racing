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
find_package(rcl_interfaces REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  std_msgs
  rcl_interfaces
)

# Include directories
include_directories(include)

# Add the Drive Node executable
add_executable(drive src/drive.cpp)
ament_target_dependencies(drive ${dependencies})

# Add the SimulationDrive Node executable
add_executable(simulation_drive src/simulation_drive.cpp)
ament_target_dependencies(simulation_drive ${dependencies})

# Add the AEB executable
add_executable(aeb src/aeb.cpp)
ament_target_dependencies(aeb ${dependencies}) 

install(TARGETS
  aeb
  drive
  simulation_drive
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()