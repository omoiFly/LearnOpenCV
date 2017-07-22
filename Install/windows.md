# 在Windows下安装OpenCV

> opencv.org提供了Windows的安装包，其中包含了为 Visual Studio 编译好的库，网上导入工程的教程很多，这里只讲用CMake-GUI 调用 MinGW/Cygwin 进行编译

> Windows下的安装和Linux类似  
只是把 CMake 的命令改为 GUI下进行：

### 1. 下载必备软件
* 下载 [CMake](https://cmake.org/download/)  [MinGW](https://sourceforge.net/projects/mingw-w64/) //安装时可能需要科学上网  
* 将 mingw的bin加入环境变量

### 2. 用CMake生成makefile
* 打开CMake,如图:
