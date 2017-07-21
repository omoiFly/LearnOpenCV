# 在Linux下安装OpenCV

### 1.编译源码前需要的第三方环境(以debian系为例)
    [compiler] sudo apt-get install build-essential
    [required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    [optional] sudo apt-get install python3-dev python3-numpy libtbb2 libtbb-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
### 2.用cmake生成makefile

*假设现在源码放在 ~/Downloads/opencv-3.2.0   其他模块放在 ~/Downloads/opencv_contrib 下 *
***
1. 在  ~/Downloads/opencv-3.2.0 下创建一个文件夹 build 存放编译成果
        mkdir build
        cd build
2. 用cmake生成一个makefile文件（可选:同时链接contrib模块）
        cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_EXTRA_MODULES=~/Downloads/oopencv_contrib/modules -D CMAKE_INSTALL_PREFIX=/usr/local ..
***注意:此时有可能会卡在 ICV: Downloading ippicv_linux_20151201.tgz... 处，复制以下链接到各大下载软件  https://github.com/opencv/opencv_3rdparty/blob/ippicv/master_20151201/ippicv/ippicv_linux_20151201.tgz  ，覆盖~/Downloads/opencv-3.2.0/3rdparty/ippicv/downloads/linux-808b791a6eac9ed78d32a7666804320e/ 下同名文件即可 (如果卡在其它地方 在这个库 https://github.com/opencv/opencv_3rdparty
的分支中找就行)***

### 3.然后进行编译链接和安装(-jX X 代表线程数)
        make -j4
        sudo make install
### 4. 最后找个模板测试一下就大功告成啦
