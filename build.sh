#!/bin/bash
# Program : Program helps to build thirdparty of the slambook2 to user's environment
# History : 2021/07/16 First release

sudo apt-get update

pip install -r requirements.txt

if [ -d "./3rdparty" ]
then
    echo "3rdparty exists!"
else
    mkdir 3rdparty
fi
cd ./3rdparty

# Dependency
sudo apt-get install libssl-dev
sudo apt- install unzip
sudo apt-get install build-essential cmake
sudo apt-get install libboost-all-dev
sudo apt-get install libglew-dev
sudo apt-get install libsuitesparse-dev
sudo apt-get install libqglviewer-dev
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libpcl-dev pcl-tools
sudo apt-get install doxygen
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common    
sudo apt-get install libflann1.8 libflann-dev
sudo apt-get install libqhull* libgtest-dev  
sudo apt-get install freeglut3-dev pkg-config  
sudo apt-get install libxmu-dev libxi-dev   
sudo apt-get install mono-complete  
sudo apt-get install qt-sdk openjdk-8-jdk openjdk-8-jre
sudo apt-get install python-argparse
sudo apt-get install python3-tk

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
    cd ~/learn_slambook2/3rdparty
fi

# Pangolin
if [ -d "/usr/local/include/pangolin" ]
then
    echo "pangolin has installed"
else
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed pangolin"
    cd ~/learn_slambook2/3rdparty
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
    cd ~/learn_slambook2/3rdparty
fi

# Sophus
if [ -d "/usr/local/include/sophus" ]
then
    echo "sophus has installed"
else
    git clone http://github.com/strasdat/Sophus.git
    cd Sophus
    mkdir build && cd build
    cmake ..
    sudo make install
    echo "Successfully installed sophus"
    cd ~/learn_slambook2/3rdparty
fi

# OpenCV 3.4.3
if [ -d "/usr/local/include/opencv" ]
then
    echo "OpenCV has installed"
else
    wget https://github.com/opencv/opencv/archive/3.4.3.zip
    unzip 3.4.3.zip
    rm 3.4.3.zip
    cd opencv-3.4.3
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed OpenCV 3.4.3"
    cd ~/learn_slambook2/3rdparty
fi

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
    cd ~/learn_slambook2/3rdparty
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
    echo "Successfully installed gflags"
    cd ~/learn_slambook2/3rdparty
fi

# glog
if [ -d "/usr/local/include/glog" ]
then
    echo "glog has installed"
else
    git clone https://github.com/google/glog
    cd glog
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed glog"
    cd ~/learn_slambook2/3rdparty
fi

# ceres-solver
if [ -d "/usr/local/include/ceres" ]
then
    echo "ceres-solver has installed"
else
    git clone https://github.com/ceres-solver/ceres-solver.git
    cd ceres-solver
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed ceres-solver"
    cd ~/learn_slambook2/3rdparty
fi

# DBoW3
if [ -d "/usr/local/include/DBoW3" ]
then
    echo "DBoW3 has installed"
else
    git clone https://github.com/rmsalinas/DBow3.git
    cd DBoW3
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed DBoW3"
    cd ~/learn_slambook2/3rdparty
fi

# vtk 8.2.0
if [ -d "/usr/local/include/vtk" ]
then
    echo "vtk has installed"
else
    wget https://www.vtk.org/files/release/8.2/VTK-8.2.0.zip
    unzip VTK-8.2.0.zip
    rm VTK-8.2.0.zip
    cd VTK-8.2.0
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed vtk 8.2.0"
    cd ~/learn_slambook2/3rdparty
fi

# pcl 1.9.1
if [ -d "/usr/local/include/pcl" ]
then
    echo "pcl has installed"
else
    wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.9.1.zip
    unzip pcl-1.9.1.zip
    rm pcl-1.9.1.zip
    cd pcl-pcl-1.9.1
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed pcl 1.9.1"
    cd ~/learn_slambook2/3rdparty
fi

# octomap
if [ -d "/usr/local/include/octomap" ]
then
    echo "octomap has installed"
else
    git clone https://github.com/OctoMap/octomap.git
    cd octomap
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    echo "Successfully installed octomap"
    cd ~/learn_slambook2/3rdparty
fi
