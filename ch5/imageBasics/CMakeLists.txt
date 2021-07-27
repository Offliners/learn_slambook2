cmake_minimum_required(VERSION 2.8)

project(imageBasics)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics imageBasics.cpp)
add_executable(undistortImage undistortImage.cpp)

# 連接OpenCV函式庫
target_link_libraries(imageBasics ${OpenCV_LIBS})
target_link_libraries(undistortImage ${OpenCV_LIBS})
