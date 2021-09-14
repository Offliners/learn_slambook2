cmake_minimum_required(VERSION 3.0)

project(examples)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})
target_link_libraries( trajectoryError Sophus::Sophus )
