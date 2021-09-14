cmake_minimum_required(VERSION 3.0)

project(useSophus)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(useSophus useSophus.cpp)
target_link_libraries( useSophus Sophus::Sophus )
