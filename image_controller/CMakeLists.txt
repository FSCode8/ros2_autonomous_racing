cmake_minimum_required(VERSION 3.8)
project(image_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_recorder src/image_recorder.cpp)
ament_target_dependencies(image_recorder rclcpp rosbag2_cpp sensor_msgs)

install(TARGETS
  image_recorder
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(image_bag_reader src/image_bag_reader.cpp)
ament_target_dependencies(image_bag_reader rclcpp rosbag2_transport sensor_msgs)

install(TARGETS
  image_bag_reader
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(image_transformer src/image_transformer.cpp)
ament_target_dependencies(image_transformer rclcpp rosbag2_transport sensor_msgs cv_bridge OpenCV)

install(TARGETS
  image_transformer
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  image_controller/image_logic.py
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
