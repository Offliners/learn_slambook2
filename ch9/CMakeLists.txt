cmake_minimum_required(VERSION 2.8)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -std=c++14")

list(APPEND CMAKE_MODULE_PATH ~/learn_slambook2/3rdparty/g2o/cmake_modules)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)
Find_Package(fmt REQUIRED)

SET(FMT_LIBRARIES fmt::fmt)
SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

add_library(bal_common common.cpp)
add_executable(bundle_adjustment_g2o bundle_adjustment_g2o.cpp)
add_executable(bundle_adjustment_ceres bundle_adjustment_ceres.cpp)

target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common)
target_link_libraries(bundle_adjustment_g2o Sophus::Sophus ${G2O_LIBS} ${FMT_LIBRARIES} bal_common)