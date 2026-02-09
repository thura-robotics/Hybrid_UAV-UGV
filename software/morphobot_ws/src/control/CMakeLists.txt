cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)



# Install Python modules
install(DIRECTORY control/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  control/mode_manager_node.py
  control/ugv_control_node.py
  control/morphing_control_node.py
  control/px4_rc_bridge.py
  control/ugv_motor_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


