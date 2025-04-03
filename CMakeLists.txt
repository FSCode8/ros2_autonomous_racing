cmake_minimum_required(VERSION 3.8)
project(ros2_autonomous_racing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  #set(_pytest_tests
  #  tests/
  #)
  #foreach(_test_path ${_pytest_tests})
  #  get_filename_component(_test_name ${_test_path} NAME_WE)
  #  ament_add_pytest_test(${_test_name} ${_test_path}
  #    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
  #    TIMEOUT 60
  #    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  #  )
  #endforeach()
  
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})
