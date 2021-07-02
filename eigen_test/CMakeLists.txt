cmake_minimum_required(VERSION 3.0.0)
project(eigen_test)

find_package(Eigen3 REQUIRED)


add_executable(main main.cpp)
