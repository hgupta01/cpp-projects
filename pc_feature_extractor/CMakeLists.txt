cmake_minimum_required(VERSION 3.1)
project(feature_extractor)

set( OpenCV_DIR "/home/himanshu/.local/lib/cmake/opencv4" )
find_package(OpenCV 4.2.0 REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

add_executable(feature_extractor main.cpp)
target_link_libraries(feature_extractor ${OpenCV_LIBS} ${PCL_LIBRARIES})