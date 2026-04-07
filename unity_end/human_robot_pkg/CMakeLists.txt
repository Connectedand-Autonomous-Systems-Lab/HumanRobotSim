cmake_minimum_required(VERSION 3.8)
project(human_robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrontierMsg.msg"
  "msg/FrontierArrayMsg.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(
  DIRECTORY human_robot_pkg/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__init__.py" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  resource/${PROJECT_NAME}
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  msg
  param
  results
  rosbag
  rviz
  tile_centers
  urdf
  DESTINATION share/${PROJECT_NAME}
)

set(_console_script_names
  map_merge_node
  frontier_detector
  wavefront_frontier_publisher
  simple_navigator
  param_loader
  map_logger
  odom_publisher
  image_visualiser
  point_to_nav_goal
  scan_limiter
  frontier_navigator
  checker
  interest_region_frontier_filter
)

set(_console_script_modules
  human_robot_pkg.merge_map
  human_robot_pkg.frontier_detector
  human_robot_pkg.wavefront_frontier_publisher
  human_robot_pkg.simple_navigator
  human_robot_pkg.param_loader
  human_robot_pkg.map_logger
  human_robot_pkg.odom_publisher
  human_robot_pkg.image_subscriber
  human_robot_pkg.point_to_nav_goal
  human_robot_pkg.scan_limiter
  human_robot_pkg.frontier_navigator
  human_robot_pkg.check_nav2_slam_ready
  human_robot_pkg.interest_region_frontier_filter
)

foreach(_script_name _module_name IN ZIP_LISTS _console_script_names _console_script_modules)
  set(_script_path "${CMAKE_CURRENT_BINARY_DIR}/${_script_name}")
  file(WRITE "${_script_path}"
"#!/usr/bin/env python3
import os
import sys

_script_dir = os.path.dirname(os.path.realpath(__file__))
_rosidl_py_path = os.path.join(_script_dir, 'rosidl_generator_py')
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
if _rosidl_py_path not in sys.path:
    sys.path.append(_rosidl_py_path)

from ${_module_name} import main

if __name__ == '__main__':
    main()
")
  file(CHMOD "${_script_path}"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
  install(PROGRAMS
    "${_script_path}"
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
