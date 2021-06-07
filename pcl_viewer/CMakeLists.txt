cmake_minimum_required(VERSION 3.1)
project(pclviewer)


find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main main.cpp)
target_link_libraries(main ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES})
