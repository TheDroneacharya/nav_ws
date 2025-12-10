cmake_minimum_required(VERSION 3.8)
project(warehouse_drone_commander)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(warehouse_drone_interfaces REQUIRED)

add_executable(
  offboard_commander_node src/offboard_commander_node.cpp
)
ament_target_dependencies(offboard_commander_node
  "rclcpp"
  "rclcpp_action"
  "mavros_msgs"
  "nav_msgs"
  "geometry_msgs"
  "warehouse_drone_interfaces"
)
target_include_directories(offboard_commander_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS offboard_commander_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  setpoint_publisher_node src/setpoint_publisher_node.cpp
)
ament_target_dependencies(setpoint_publisher_node
  "rclcpp"
  "mavros_msgs"
)
install(TARGETS setpoint_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  go_to_pose_action_server_node src/go_to_pose_action_server_node.cpp
)
ament_target_dependencies(go_to_pose_action_server_node
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "mavros_msgs"
  "warehouse_drone_interfaces"
)
target_include_directories(go_to_pose_action_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS go_to_pose_action_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src launch config
  DESTINATION share/${PROJECT_NAME}
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
