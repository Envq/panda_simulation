#!/bin/bash

echo "Install all dependencies"
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libboost-filesystem-dev clang-format



echo "Prepare workspace"

mkdir -p ~/panda_ws/src

cd ../

mv panda_controller ~/panda_ws/src

git clone https://github.com/erdalpekel/panda_moveit_config.git

git clone --branch simulation https://github.com/erdalpekel/franka_ros.git



echo "Install libfranka"

cd

mkdir -p github

cd ~/github

git clone --recursive https://github.com/frankaemika/libfranka

mkdir libfranka/build

cd libfranka/build

cmake -DCMAKE_BUILD_TYPE=Release ..

cmake --build .



echo "Configure workspace"

cd ~/panda_ws

rosdep install --from-paths src --ignore-src -y --skip-keys libfranka

catkin config -j$(nproc) --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$HOME/github/libfranka/build

catkin init



echo "Build"

cd ~/panda_ws

catkin build

echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc

echo 'source ~/panda_ws/devel/setup.bash' >> ~/.bashrc