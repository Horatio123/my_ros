cmake_minimum_required(VERSION 3.5)
project(tree_test)

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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include_directories(
#   include
# )

add_library(bt_sample_nodes STATIC src/dummy_node.cpp)
target_include_directories(bt_sample_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bt_sample_nodes behaviortree_cpp_v3 rclcpp)

add_library(decorator_nodes STATIC src/rate_controller.cpp)
target_include_directories(decorator_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(decorator_nodes behaviortree_cpp_v3)

add_library(control_nodes STATIC src/pipeline_sequence.cpp)
target_include_directories(control_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(control_nodes behaviortree_cpp_v3)


add_executable(tree_test src/tree_test.cpp)
target_include_directories(tree_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tree_test behaviortree_cpp_v3)
target_link_libraries(tree_test bt_sample_nodes decorator_nodes control_nodes)

install(TARGETS tree_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS bt_sample_nodes
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS decorator_nodes
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS control_nodes
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
