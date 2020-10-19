#!/bin/bash

cd ~/
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
sudo rosdep init
rosdep update
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator desktop --rosdistro noetic --deps --tar > noetic-desktop.rosinstall
mkdir ./src
vcs import --input noetic-desktop.rosinstall ./src
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3
