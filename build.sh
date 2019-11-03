#!/bin/bash
echo "###############################"
echo "##  Install all dependencies  #"
echo "###############################"
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libboost-filesystem-dev clang-format ros-melodic-catkin python-catkin-tools



echo "#######################"
echo "##  Create workspace  #"
echo "#######################"


echo "##  Move panda_simulation directory  ##"
mkdir -p ~/panda_ws/src

mv panda_simulation ~/panda_ws/src

cd ~/panda_ws/src

echo "##  Get dependencies from github  ##"

git clone https://github.com/erdalpekel/panda_moveit_config.git

git clone --branch simulation https://github.com/erdalpekel/franka_ros.git

echo "##  Install libfranka  ##"

mkdir -p ~/github

cd ~/github

git clone --recursive https://github.com/frankaemika/libfranka

mkdir -p libfranka/build

cd libfranka/build

cmake -DCMAKE_BUILD_TYPE=Release ..

cmake --build .



echo "##########################"
echo "##  Configure workspace  #"
echo "##########################"

echo 'export ROS_OS_OVERRIDE=ubuntu:18.04:bionic' >> ~/.bashrc

echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc

echo 'source ~/panda_ws/devel/setup.bash' >> ~/.bashrc

source ~/.bashrc

cd ~/panda_ws

rosdep install --from-paths src --ignore-src -y --skip-keys libfranka

catkin config -j$(nproc) --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$HOME/github/libfranka/build

catkin init



echo "######################"
echo "##  Build workspace  #"
echo "######################"

catkin build