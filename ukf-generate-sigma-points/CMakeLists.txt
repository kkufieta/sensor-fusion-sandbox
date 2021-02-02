# original: cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(ukf-generate-sigma-points)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)


# Executable for create matrix exercise
add_executable (gen-sigma-points src/main.cpp src/ukf.cpp)
target_link_libraries (gen-sigma-points Eigen3::Eigen)