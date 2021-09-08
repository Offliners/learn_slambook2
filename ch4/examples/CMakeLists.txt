cmake_minimum_required(VERSION 3.0)

project(examples)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories( "/usr/local/include/eigen3" )
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
find_package(Sophus REQUIRED)
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})
target_link_libraries( trajectoryError Sophus::Sophus )
