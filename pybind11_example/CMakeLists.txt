cmake_minimum_required(VERSION 3.5)
project(pybind11_example)

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
find_package(pybind11 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

pybind11_add_module(pybind_cpp_data_processor 
    src/pybind11_data_processor.cpp
    src/data_processor.cpp
)
pybind11_add_module(pybind_simple_function src/simple_function.cpp)

# ament_target_dependencies(pybind_cpp_data_processor
#   pybind11
# )
target_include_directories(pybind_cpp_data_processor PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

get_property(_pybind_include_directories TARGET pybind_cpp_data_processor PROPERTY INCLUDE_DIRECTORIES)
message(STATUS "pybind include directories: ${_pybind_include_directories}")

install(TARGETS pybind_cpp_data_processor
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pybind_simple_function
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
