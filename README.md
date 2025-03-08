# Design and Implementation of an Autonomous Drone System

## 环境配置与安装指南

### 一、依赖安装

#### 1. 安装 CMake

```bash
sudo apt-get install cmake
```

#### 2. 安装 google-glog 和 gflags

```bash
sudo apt-get install libgoogle-glog-dev libgflags-dev
```

#### 3. 安装 Eigen3

```bash
sudo apt-get install libeigen3-dev
```

### 二、OpenCV 安装（版本：3.4.16）

#### 1. 下载 OpenCV 源码

从 [OpenCV 官网](https://opencv.org/releases/) 下载 OpenCV 3.4.16 的源码压缩包。

#### 2. 安装编译工具与依赖

```bash
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
gfortran openexr libatlas-base-dev python3-dev python3-numpy \
libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
```

#### 3. 编译与安装 OpenCV

```bash
mkdir ~/opencv_build && cd ~/opencv_build
mv ~/Downloads/opencv-3.4.16.zip ./
unzip opencv-3.4.16.zip
cd ~/opencv_build/opencv-3.4.16
mkdir -p build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D BUILD_EXAMPLES=ON \
      -D CMAKE_INSTALL_PREFIX=/usr/local ..

make -j$(nproc)
sudo make install
```

### 三、运行项目

#### 编译并执行 C++ 文件

编译：
```bash
g++ src/train.cpp -o train
```

运行：
```bash
./train
```

或直接使用一行命令编译并执行：

```bash
g++ src/train.cpp -o train && ./train
```


