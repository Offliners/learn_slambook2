cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(imageBasics)
add_subdirectory(stereo)
add_subdirectory(rgbd)
