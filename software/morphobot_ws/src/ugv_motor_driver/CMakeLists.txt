cmake_minimum_required(VERSION 3.8)
project(ugv_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReadPositions.srv"
  "srv/WritePositions.srv"
  "srv/WriteVelocities.srv"
  "srv/ReadVelocities.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Create C++ hardware interface library
add_library(
  ${PROJECT_NAME}_plugin
  SHARED
  src/st3215_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}_plugin
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}_plugin
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link service interfaces to the plugin
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_plugin "${cpp_typesupport_target}")

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface st3215_hardware_interface.xml)

# Install C++ library
install(
  TARGETS ${PROJECT_NAME}_plugin
  DESTINATION lib
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install Python modules (scripts directory structure)
install(
  DIRECTORY nodes/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  nodes/st3215_service_node.py
  nodes/motor_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin description
install(
  FILES st3215_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rosidl_default_runtime
  rclpy
)

ament_package()
