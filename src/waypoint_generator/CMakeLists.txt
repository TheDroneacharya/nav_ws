cmake_minimum_required(VERSION 3.20)
project(waypoint_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(waypoint_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(waypoint_server src/waypoint_server.cpp)
ament_target_dependencies(waypoint_server
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  std_msgs
  waypoint_interfaces
  ament_index_cpp
)
target_link_libraries(waypoint_server yaml-cpp)

install(TARGETS waypoint_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/demo_simulation.py
  DESTINATION lib/${PROJECT_NAME}
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
