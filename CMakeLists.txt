cmake_minimum_required(VERSION 3.5)
project(me314)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(xarm_planner REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_moveit_config REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# Install the Python 'me314' package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  me314/xarm_api_scripts/move_A_to_B.py
  me314/teleoperation/xarm_spacemouse_ros2.py
  me314/teleoperation/list_hid.py
  me314/drivers/install_spacemouse.py
  me314/xarm_planner/xarm_test_move_script.py
  me314/xarm_planner/xarm_commander_node.py
  me314/xarm_planner/xarm_a2b_example.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
)

# Install any Gazebo models
install(
  DIRECTORY me314/gazebo_models
  DESTINATION share/${PROJECT_NAME}/me314
  PATTERN "__pycache__" EXCLUDE
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
