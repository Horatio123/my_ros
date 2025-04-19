cmake_minimum_required(VERSION 3.5)
project(read_bag)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# 打印各组件路径信息
message(STATUS "ament_cmake include dir: ${ament_cmake_INCLUDE_DIRS}")
message(STATUS "ament_cmake libraries: ${ament_cmake_LIBRARIES}")
message(STATUS "rclcpp include dir: ${rclcpp_INCLUDE_DIRS}")
message(STATUS "rclcpp libraries: ${rclcpp_LIBRARIES}")
message(STATUS "rosbag2_cpp include dir: ${rosbag2_cpp_INCLUDE_DIRS}")
message(STATUS "rosbag2_cpp libraries: ${rosbag2_cpp_LIBRARIES}")
message(STATUS "geometry_msgs include dir: ${geometry_msgs_INCLUDE_DIRS}")
message(STATUS "geometry_msgs libraries: ${geometry_msgs_LIBRARIES}")
message(STATUS "nlohmann_json include dir: ${nlohmann_json_INCLUDE_DIRS}")

add_executable(read_bag read_bag.cpp)
ament_target_dependencies(read_bag
    rclcpp
    rosbag2_cpp
    geometry_msgs
    nlohmann_json
)

install(TARGETS read_bag DESTINATION lib/${PROJECT_NAME})