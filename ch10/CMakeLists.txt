cmake_minimum_required(VERSION 2.8)
project(ch10)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O2")

list(APPEND CMAKE_MODULE_PATH ~/learn_slambook2/3rdparty/g2o/cmake_modules)

# Eigen
find_package(Eigen3 REQUIRED)

# sophus 
find_package(Sophus REQUIRED)

# fmt
Find_Package(fmt REQUIRED)

# g2o 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

SET(FMT_LIBRARIES fmt::fmt)

add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3 g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES} ${FMT_LIBRARIES})

add_executable(pose_graph_g2o_lie pose_graph_g2o_lie_algebra.cpp)
target_link_libraries(pose_graph_g2o_lie g2o_core g2o_stuff ${CHOLMOD_LIBRARIES} Sophus::Sophus ${FMT_LIBRARIES})