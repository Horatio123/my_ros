cmake_minimum_required(VERSION 3.5)
project(thread_test)

find_package(Threads REQUIRED)
add_executable(thread_test src/thread_test.cpp)
target_link_libraries(thread_test Threads::Threads)
