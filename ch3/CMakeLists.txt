cmake_minimum_required(VERSION 2.8)
project(ch3)

set(CMAKE_CXX_FLAGS "-std=c++11")

# 加入各個子工程
add_subdirectory(useEigen)
add_subdirectory(useGeometry)
add_subdirectory(visualizeGeometry)
add_subdirectory(examples)
add_subdirectory(exercise)
