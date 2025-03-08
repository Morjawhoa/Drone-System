# Design and Implementation of an Autonomous Drone System

## Installation Requirements

### 1. Install Eigen3
Execute the following commands to install required dependencies:

```bash
# Install CMake
sudo apt-get install cmake

# Install google-glog and gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev

# Install Eigen3
sudo apt-get install libeigen3-dev
```

### 2. Install OpenCV 3.4.16

#### Step a: Download OpenCV
Download OpenCV version 3.4.16 from the official release page: [OpenCV Releases](https://opencv.org/releases/)

#### Step b: Install Dependencies
Execute the following command to install all necessary dependencies and build tools:

```bash
sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
gfortran openexr libatlas-base-dev python3-dev python3-numpy \
libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
```

#### Step c: Compile OpenCV
Run the following commands to compile the OpenCV source code:

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
```

#### Step d: Installation

Check the number of available CPU threads with:
```bash
nproc
```

Then compile and install OpenCV using:
```bash
make -j4
sudo make install
```

