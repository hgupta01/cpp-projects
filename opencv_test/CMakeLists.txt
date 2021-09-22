cmake_minimum_required(VERSION 3.0)
project(surfmodule)

set(CMAKE_SHARED_MODULE_PREFIX "")

# Find necessary packages
find_package(PythonInterp 3)
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIR})

set( OpenCV_DIR "/home/himanshu/.local/lib/cmake/opencv4" )
find_package(OpenCV 4.2.0 REQUIRED)
 
# find_package(Boost COMPONENTS python3 REQUIRED)
find_package(Boost COMPONENTS python${PYTHON_VERSION_MAJOR})

include_directories(include ${Boost_INCLUDE_DIR})
 
# Build & Link our library
add_library(surfmodule MODULE surf_descriptor.cpp)

# setting Boost_LIBRARIES & PYTHON_LIBRARIES
if(UNIX)
        set(Boost_LIBRARIES "/usr/lib/x86_64-linux-gnu/libboost_python3-py36.so.1.65.1")
        set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.6m.so")
endif()

target_link_libraries(surfmodule PRIVATE ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
