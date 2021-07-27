# Slambook2 Note
code of 14 lectures on visual SLAM 2nd built on Windows 10 using WSL(Ubuntu 16.04)

## Thirdparty
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

### Sophus
照以下步驟安裝
```shell
git clone http://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
sudo make install
```
書中程式碼使用`SE3d`與`SO3d`，因為Sophus改成用模板類別，因此改成`SE3`與`SO3`

若使用`SE3`需要`#include<sophus/se3.h>`

若使用`SO3`需要`#include<sophus/so3.h>`

## IDE
Visual studio code 

## Visual Studio Code Extensions  
* C/C++ : [Link](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
* Remote - WSL : [Link](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl)

## Reference
* github : https://github.com/gaoxiang12/slambook2
* SLAM視覺十四講：雙倍內容強化版 (`ISBN：9789865501044`)

![Reference](https://github.com/Offliners/SlambookWSL/blob/main/reference.png)
