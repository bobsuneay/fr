cmake_minimum_required(VERSION 3.8)
project(ros2_hkv_gripper)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  std_srvs
  hardware_interface
  pluginlib
  controller_interface
  rosidl_default_runtime
)

# ROS 2 interfaces (messages/services)
set(interface_files
  "msg/GripperRegisters.msg"
  "srv/GripperCommand.srv"
  "srv/ReadFingerState.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)

# Gripper controller library (driver + node)
add_library(
  ros2_hkv_gripper_core
  SHARED
  src/serial.cpp
  src/default_serial.cpp
  src/modbus_utils.cpp
  src/default_driver.cpp
  src/default_driver_factory.cpp
  src/gripper_controller_node.cpp
  src/gripper_hardware_interface.cpp
)

target_include_directories(
  ros2_hkv_gripper_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ros2_hkv_gripper_core
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ros2_hkv_gripper_core ${cpp_typesupport_target})

# Plugin export for hardware_interface
pluginlib_export_plugin_description_file(hardware_interface gripper_hardware_interface_plugin.xml)

# Activation controller library
add_library(
  ros2_hkv_gripper_activation
  SHARED
  src/gripper_activation_controller.cpp
)

target_include_directories(
  ros2_hkv_gripper_activation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ros2_hkv_gripper_activation
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Plugin export for controller_interface
pluginlib_export_plugin_description_file(controller_interface gripper_controller_plugin.xml)

# Gripper controller node executable
add_executable(ros2_hkv_gripper_node
  src/main.cpp
)

target_link_libraries(
  ros2_hkv_gripper_node
  ros2_hkv_gripper_core
)

# Install targets
install(
  TARGETS ros2_hkv_gripper_core ros2_hkv_gripper_activation
  EXPORT ros2_hkv_gripper_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ros2_hkv_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config urdf meshes rviz nodes
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS
    nodes/rviz_gripper_cmd_marker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rviz_gripper_cmd_marker
)

install(
  FILES gripper_hardware_interface_plugin.xml gripper_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_targets(ros2_hkv_gripper_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
