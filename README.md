# Slambook2 Note
code of 14 lectures on visual SLAM 2nd built on Windows 10 using WSL(Ubuntu 16.04)

## Table of Content
* [Thirdparty](#thirdparty)
  * [CMake](#cmake)
  * [Eigen](#eigen)
  * [Pangolin](#pangolin)
  * [fmt](#fmt)
  * [Sophus](#sophus)
  * [OpenCV](#opencv)
  * [Boost](#boost)
* [DEMO](#demo)
  * [trajectoryError](#trajectoryerror) 
* [IDE](#ide)
* [References](#references) 

## Thirdparty
### CMake
Ubuntu 16.0 預設Cmake版本是3.5，但版本過舊，所以使用Cmake編譯一些第三方函式庫時會有問題，因此建議使用以下指令來更新到3.16以上
```shell
sudo apt-get install build-essential
wget http://www.cmake.org/files/v3.16/cmake-3.16.0.tar.gz
tar xf cmake-3.16.0.tar.gz
cd cmake-3.16.0
./configure
```

接著，為了解決路徑問題，所以要加到環境變數
```shell
sudo gedit ~/.bashrc
```

在.bashrc中加入以下兩行
```shell
export PATH=/usr/local/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

再來更新環境變數
```shell
source ~/.bashrc
```

最後使用以下指令即可看到新版的Cmake
```shell
$ cmake --version
cmake version 3.16.0

CMake suite maintained and supported by Kitware (kitware.com/cmake).
```

### Eigen
輸入此指令安裝，並加入到C/C++插件的includePath
```shell
apt-get install libeigen3-dev
```

### Pangolin
先用此指令安裝Glew
```shell
sudo apt-get install libglew-dev
```

再照著以下步驟安裝
```shell
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

### fmt
照以下步驟安裝
```shell
git clone  https://github.com/fmtlib/fmt.git
cd fmt
mkdir build && cd build
cmake ..
make
sudo make install
```

### Sophus
照以下步驟安裝
```shell
git clone http://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build
cmake ..
sudo make install
```
如果用到模板類的Sophus需要進行以下更改

`SE3`需要`#include<sophus/se3.h>`

`SO3`需要`#include<sophus/so3.h>`

`SE3d`與`SO3d`改成用模板類別，因此改成`SE3`與`SO3`

### OpenCV
安裝`3.4.3`，照以下步驟
```shell
sudo apt-get install build-essential cmake
wget https://github.com/opencv/opencv/archive/3.4.3.zip
unzip 3.4.3.zip
cd opencv-3.4.3
mkdir -p build && cd build
cmake ..
make
sudo make install
```

### Boost
使用此指令安裝
```shell
sudo apt-get install libboost-all-dev
```

## DEMO
### trajectoryError
![trajectoryError](https://github.com/Offliners/Slambook2_note/blob/main/demo/trajectoryError.JPG)

## IDE
Visual studio code 

* Visual Studio Code Extensions  
  * C/C++ : [Link](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
  * Remote - WSL : [Link](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl)

## References
* github : https://github.com/gaoxiang12/slambook2
* SLAM視覺十四講：雙倍內容強化版 (`ISBN：9789865501044`)

![References](https://github.com/Offliners/SlambookWSL/blob/main/reference.png)
