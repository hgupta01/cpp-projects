
cmake_minimum_required(VERSION 3.10)
project(calculatebb)


# This include only set compile options: Use it as a reference!
#include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/SetTargetCompileOptions.cmake)

### only set this variable if ApproxMVBB is not installed system-wide
set(ApproxMVBB_DIR "/home/himanshu/.local/ApproxMVBB/cmake/ApproxMVBB/cmake")
get_filename_component( ApproxMVBB_DIR ${ApproxMVBB_DIR} ABSOLUTE )
message(STATUS "ApproxMVBB_DIR:" ${ApproxMVBB_DIR})

### Add libraries
find_package(Eigen3 REQUIRED)
find_package(ApproxMVBB REQUIRED)

# Add simple executable
add_library(calculatebb SHARED calculatebb.cpp)
target_link_libraries(calculatebb ApproxMVBB::Core)

### Add simple executable
#add_executable(libraryUsage main.cpp)
#target_link_libraries(libraryUsage ApproxMVBB::Core)



