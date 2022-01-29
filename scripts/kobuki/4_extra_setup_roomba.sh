#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

name_ros_distro="indigo"
name_catkinws="catkin_ws"
name_rosbuildws="rosbuild_ws"
name_custom_bashrc="mybashrc"


start_pwd=$(pwd)

echo "---------------------------------------"
echo "----------- 4. SETUP ROOMBA -----------"
echo "---------------------------------------"
echo "---[ Creates rosbuild workspace overlaying catkin_ws ]"
echo "---[ Downloads roomba packages ]"
echo "---[ Compiles roomba packages ]"
echo "---------------------------------------"

echo "---[ Creating ~/$name_rosbuildws rosbuild workspace ]"
mkdir -p ~/${name_rosbuildws}
cd ~/${name_rosbuildws}
echo "---[ Initializing ~/$name_rosbuildws rosbuild workspace with ~/$name_catkinws catkin workspace ]"
rosws init . ~/${name_catkinws}/devel

cd ~/${name_rosbuildws}
echo "---[ Downloading roomba_500_series and cereal_port packages ]"
wstool set -y roomba_500_series --svn https://isr-uc-ros-pkg.googlecode.com/svn/stacks/roomba_robot/trunk/roomba_500_series
wstool set -y cereal_port --svn https://isr-uc-ros-pkg.googlecode.com/svn/stacks/serial_communication/trunk/cereal_port
wstool update cereal_port roomba_500_series
rosdep update
source ~/${name_rosbuildws}/setup.bash
echo "---[ Compiling roomba_500_series and cereal_port packages ]"
rosmake cereal_port roomba_500_series

echo "---[ Adding user kobuki to dialout group ]"
sudo adduser kobuki dialout

echo "---[ Adding source of ~/$name_rosbuildws rosbuild workspace to ~/.$name_custom_bashrc  ]"
if ! grep -q "source ~/$name_rosbuildws/setup.bash" ~/.${name_custom_bashrc}; then
  echo "source ~/$name_rosbuildws/setup.bash" >> ~/.${name_custom_bashrc}
fi

####################### FINISH #######################

echo "---[ Script Complete ]"

cd $start_pwd

# exec bash

return 0

