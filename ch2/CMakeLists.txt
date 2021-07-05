# 聲明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 聲明一個 cmake 工程
project(HelloSLAM)

# 設置編譯模式
set(CMAKE_BUILD_TYPE "Debug")

# 添加一個可執行程序
# 語法：add_executable( 程序名 源代碼文件 ）
add_executable(helloSLAM helloSLAM.cpp)

# 添加Hello庫
add_library(hello libHelloSLAM.cpp)

# 共享庫
add_library(hello_shared SHARED libHelloSLAM.cpp)

# 添加可執行程序調用hello庫中函數
add_executable(useHello useHello.cpp)

# 將庫文件鏈接到可執行程序上
target_link_libraries(useHello hello_shared)
