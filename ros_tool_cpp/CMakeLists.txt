cmake_minimum_required(VERSION 3.5)
project(ros_tool_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(laser_geometry REQUIRED)

add_executable(pointcloud2_to_pcd src/pointcloud2_to_pcd.cpp)
ament_target_dependencies(pointcloud2_to_pcd
  rclcpp
  rosbag2_cpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

add_executable(laserscan_to_pcd
    src/laserscan_to_pcd.cpp
)
ament_target_dependencies(laserscan_to_pcd
    rclcpp 
    rosbag2_cpp 
    sensor_msgs 
    pcl_conversions
    pcl_ros
    laser_geometry
)

install(TARGETS
  pointcloud2_to_pcd
  laserscan_to_pcd
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()