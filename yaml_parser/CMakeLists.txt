cmake_minimum_required(VERSION 3.1)
project(pclviewer)


find_package(yaml-cpp REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main ${YAML_CPP_LIBRARIES})
