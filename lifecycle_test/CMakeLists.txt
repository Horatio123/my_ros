cmake_minimum_required(VERSION 3.5)
project(lifecycle_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(/home/ros/rclcpp/install/rclcpp_lifecycle/include)
# include_directories(/opt/ros/foxy/include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lifecycle_node src/lifecycle_node.cpp)
ament_target_dependencies(lifecycle_node rclcpp rclcpp_lifecycle)

add_executable(moody_lifecycle_node src/moody_lifecycle_node.cpp)
ament_target_dependencies(moody_lifecycle_node rclcpp rclcpp_lifecycle)

add_executable(lifecycle_pb src/lifecycle_pb.cpp)
ament_target_dependencies(lifecycle_pb rclcpp rclcpp_lifecycle)

add_executable(lifecycle_client src/lifecycle_client.cpp)
ament_target_dependencies(lifecycle_client rclcpp rclcpp_lifecycle lifecycle_msgs)

# target_include_directories(lifecycle_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;/opt/ros/foxy/include>
#   $<INSTALL_INTERFACE:include>)
# 添加依赖库
# target_link_libraries(lifecycle_node
#   rclcpp
#   rclcpp_lifecycle
# )

install(TARGETS lifecycle_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS moody_lifecycle_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lifecycle_pb
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lifecycle_client
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
