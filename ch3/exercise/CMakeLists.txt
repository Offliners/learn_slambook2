cmake_minimum_required( VERSION 2.8 )
project( exercise )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable( q5 q5.cpp )
add_executable( q6 q6.cpp )
