## Coverage Active SLAM

![](https://github.com/RicheyHuang/CoverageActiveSLAM/blob/main/image/robot.png)
![](https://github.com/RicheyHuang/CoverageActiveSLAM/blob/main/image/scene1.png)
![](https://github.com/RicheyHuang/CoverageActiveSLAM/blob/main/image/scene2.png)
![](https://github.com/RicheyHuang/CoverageActiveSLAM/blob/main/image/house.png)
![](https://github.com/RicheyHuang/CoverageActiveSLAM/blob/main/image/exploration.png)
![](https://github.com/RicheyHuang/CoverageActiveSLAM/blob/main/image/coverage.PNG)


### 0. Overview

A SLAM framework which automatically covers the area with obstacle avoidance, using gazebo for simulation.

This Project is used for study, not for commercial uses.

Tested on Ubuntu 18.04, with ROS Melodic and Gazebo 9.



### 1. Dependencies

OpenCV

CGAL

Cartographer

ROS-Navigation

Gazebo

Abseil

Protobuf

Ceres Solver



### 2. Installation

```bash
git clone https://github.com/RicheyHuang/CoverageActiveSLAM.git


# Install ROS-Melodic

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo apt-get install ros-melodic-navigation

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash

# Install the required libraries
sudo apt-get update
sudo apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    ninja-build \
    stow
    
# Install CGAL    
sudo apt-get install libcgal-dev

cd CoverageActiveSLAM/3rd_party

# Build and install abseil
cd abseil-cpp
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=../stow/absl \
  ..
ninja
sudo ninja install
cd ../stow
sudo stow absl

cd ../..

# Build and install Ceres.
cd ceres-solver
mkdir build
cd build
cmake .. -G Ninja -DCXX11=ON -DCMAKE_INSTALL_PREFIX=../install
ninja
CTEST_OUTPUT_ON_FAILURE=1 ninja test
sudo ninja install

cd ../..

# Build and install proto3.
cd protobuf
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  -DCMAKE_INSTALL_PREFIX=../install \
  ../cmake
ninja
sudo ninja install

cd ../../..

sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build stow

# Install gazebo 9
sudo apt-get install gazebo9
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-gazebo-ros-pkgs
sudo apt upgrade


source /opt/ros/melodic/setup.bash
cd simulation_stack/src
catkin_init_workspace
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES='custom_msgs;custom_srvs'
catkin_make -DCATKIN_WHITELIST_PACKAGES=''

source devel/setup.bash
cd ../slam_stack/src
catkin_init_workspace
cd ..
catkin_make_isolated --install --use-ninja

source install_isolated/setup.bash
cd ../navigation_stack/src
catkin_init_workspace
cd ..
catkin_make

source devel/setup.bash
cd ../settings/src
catkin_init_workspace
cd ..
catkin_make
cd ..

# the order of the chained overlaying workspace: ros-melodic -> simulation_stack -> slam_stack -> navigation_stack -> settings
# use "roscd PACKAGE" to check if a package can be found in current environment

```



### 3. Run a Demo

```bash
# 1. Enter the root directory of CoverageActiveSLAM
# 2. source the bash file in order
source /opt/ros/melodic/setup.bash
source simulation_stack/devel/setup.bash
source slam_stack/install_isolated/setup.bash
source navigation_stack/devel/setup.bash
source settings/devel/setup.bash
# 3. launch
roscd configuration/launch
roslaunch caslam.launch

```



### 4. How to Recompile Packages

```bash
# if you want to recompile the packages, remenber to source the bash files above first, for example:

# recompile simulation_stack:
# 1. Enter the root directory of CoverageActiveSLAM
# 2. source the bash file in order
source /opt/ros/melodic/setup.bash
source simulation_stack/devel/setup.bash
source slam_stack/install_isolated/setup.bash
source navigation_stack/devel/setup.bash
source settings/devel/setup.bash
# 3. make:
cd simulation_stack
catkin_make

# recompile navigation_stack:
# 1. Enter the root directory of CoverageActiveSLAM
# 2. source the bash file in order
source /opt/ros/melodic/setup.bash
source simulation_stack/devel/setup.bash
source slam_stack/install_isolated/setup.bash
source navigation_stack/devel/setup.bash
source settings/devel/setup.bash
# 3. make:
cd navigation_stack
catkin_make

# recompile settings:
# 1. Enter the root directory of CoverageActiveSLAM
# 2. source the bash file in order
source /opt/ros/melodic/setup.bash
source simulation_stack/devel/setup.bash
source slam_stack/install_isolated/setup.bash
source navigation_stack/devel/setup.bash
source settings/devel/setup.bash
# 3. make:
cd settings
catkin_make

# recompile slam_stack:
# 1. Enter the root directory of CoverageActiveSLAM
# 2. source the bash file in order
source /opt/ros/melodic/setup.bash
source simulation_stack/devel/setup.bash
source slam_stack/install_isolated/setup.bash
source navigation_stack/devel/setup.bash
source settings/devel/setup.bash
# 3. make:
cd slam_stack
catkin_make_isolated --install --use-ninja
```



### 5. Reference & Acknowledgements


1. ROS
2. Gazebo
3. Cartographer
https://github.com/cartographer-project/cartographer
https://github.com/cartographer-project/cartographer_ros
4. Virtual Costmap Layer
https://github.com/GMahmoud/virtual_costmap_layer
5. AWS Robomaker
https://github.com/aws-robotics/aws-robomaker-small-house-world


