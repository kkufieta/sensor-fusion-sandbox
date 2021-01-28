# original: cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(kalman-lidar)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)


# Executable for create matrix exercise
add_executable (kalman_lidar src/main.cpp src/kalman_filter.cpp src/tracking.cpp src/measurement_package.h)
target_link_libraries (kalman_lidar Eigen3::Eigen)