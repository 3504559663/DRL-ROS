cmake_minimum_required(VERSION 3.8)
project(pointcloud_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

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

add_executable(pointcloud_merger_node src/pointcloud_merger_node.cpp)
add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)

ament_target_dependencies(pointcloud_merger_node 
rclcpp 
sensor_msgs 
tf2_sensor_msgs
pcl_conversions
message_filters
tf2_eigen
)

ament_target_dependencies(pointcloud_to_laserscan_node 
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
  message_filters
)

install(TARGETS pointcloud_merger_node pointcloud_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
