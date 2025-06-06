cmake_minimum_required(VERSION 3.5)
project(nav2_custom_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  visualization_msgs
  tf2_ros
  pluginlib
)

add_library(nav2_custom_controller SHARED
  src/custom_controller.cpp
)

ament_target_dependencies(nav2_custom_controller
  ${dependencies}
)

target_compile_definitions(nav2_custom_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS nav2_custom_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_custom_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

ament_package()
