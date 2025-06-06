cmake_minimum_required(VERSION 3.5)
project(ros_utils)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(utils STATIC src/node_utils.cpp src/lifecycle_node.cpp src/node_thread.cpp src/lifecycle_service_client.cpp src/string_utils.cpp src/lifecycle_utils.cpp)
target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(utils rclcpp rclcpp_lifecycle lifecycle_msgs)

add_executable(utils_test src/utils_test.cpp)
target_include_directories(utils_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(utils_test utils)

add_executable(lifecycle_node_test src/lifecycle_node_test.cpp)
target_include_directories(lifecycle_node_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lifecycle_node_test utils)

add_executable(lifecycle_service_test src/lifecycle_service_test.cpp)
target_include_directories(lifecycle_service_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lifecycle_service_test utils)

add_executable(lifecycle_up_cmd src/lifecycle_bringup_commandline.cpp)
target_include_directories(lifecycle_up_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lifecycle_up_cmd utils)



install(TARGETS utils_test
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lifecycle_node_test
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lifecycle_service_test
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lifecycle_up_cmd
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
