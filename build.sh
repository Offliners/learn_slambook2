#!/bin/bash
# Program : Program helps to build thirdparty of the slambook2 to user's environment
# History : 2021/07/16 First release

mkdir 3rdparty
cd ./3rdparty

# Eigen 3.3.9
if [ -d "/usr/local/include/eigen3" ]
then
    echo "eigen3 has installed"
else
    wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.zip
    unzip eigen-3.3.9.zip
    sudo rm eigen-3.3.9.zip
    cd eigen-3.3.9
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed Eigen 3.3.9"
    cd ../..
fi

# fmt
if [ -d "/usr/local/include/fmt" ]
then
    echo "fmt has installed"
else
    git clone https://github.com/fmtlib/fmt.git
    cd fmt
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed fmt"
    cd ../..
fi

# Sophus
if [ -d "/usr/local/include/sophus" ]
then
    echo "sophus has installed"
else
    git clone http://github.com/strasdat/Sophus.git
    cd Sophus
    mkdir build
    cd build
    cmake ..
    sudo make install
    echo "Successfully installed sophus"
    cd ../..
fi

# OpenCV 3.4.3
if [ -d "/usr/local/include/opencv" ]
then
    echo "OpenCV has installed"
else
    sudo apt-get install build-essential cmake
    wget https://github.com/opencv/opencv/archive/3.4.3.zip
    unzip 3.4.3.zip
    rm 3.4.3.zip
    cd opencv-3.4.3
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed OpenCV 3.4.3"
    cd ../..
fi

# Boost
sudo apt-get install libboost-all-dev

# g2o
if [ -d "/usr/local/include/g2o" ]
then
    echo "g2o has installed"
else
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed g2o"
    cd ../..
fi

# gflags
if [ -d "/usr/local/include/gflags" ]
then
    echo "gflags has installed"
else
    git clone https://github.com/gflags/gflags
    cd gflags
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed glags"
    cd ../..
fi

# ceres-solver
if [ -d "/usr/local/include/ceres" ]
then
    echo "g2o has installed"
else
    git clone https://github.com/ceres-solver/ceres-solver.git
    cd ceres-solver
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed ceres-solver"
    cd ../..
fi