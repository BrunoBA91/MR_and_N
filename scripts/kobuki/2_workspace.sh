#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

name_ros_distro="indigo"
name_catkinws="catkin_ws"
name_custom_bashrc="mybashrc"


start_pwd=$(pwd)

echo "---------------------------------------"
echo "-------- 2. WORKSPACE SCRIPT   --------"
echo "---------------------------------------"
echo "---[ For ROS version: $name_ros_distro ]"
echo "---[ Creates workspace in: ~/$name_catkinws ]"
echo "---[ Setups config in:   ~/.$name_custom_bashrc ]"
echo "---------------------------------------"

####################### BASHRC #######################

if ! grep -q "source ~/.${name_custom_bashrc}" ~/.bashrc; then
  echo "---[ Adding ~/.${name_custom_bashrc} source to ~/.bashrc..."

  echo "############## ROS INSTALL SCRIPT ##############" >> ~/.bashrc
  echo "# Source ROS setup in ~/.${name_custom_bashrc}" >> ~/.bashrc
  echo "if [ -f ~/.${name_custom_bashrc} ]; then" >> ~/.bashrc
  echo "  source ~/.${name_custom_bashrc}" >> ~/.bashrc
  echo "else" >> ~/.bashrc
  echo "  echo "---ERROR: missing ~./${name_custom_bashrc} file"" >> ~/.bashrc
  echo "fi" >> ~/.bashrc
  echo "############## ROS INSTALL SCRIPT ##############" >> ~/.bashrc
fi

if ! [ -f ~/.${name_custom_bashrc} ];then
  echo "############## ROS INSTALL SCRIPT ##############" >> ~/.${name_custom_bashrc}
  echo "####### ~/.${name_custom_bashrc} with ROS Setup" >> ~/.${name_custom_bashrc}
  echo "####### Sourced by ~/.bashrc" >> ~/.${name_custom_bashrc}
fi

if ! grep -q "source /opt/ros/${name_ros_distro}/setup.bash" ~/.${name_custom_bashrc}; then
  echo "source /opt/ros/${name_ros_distro}/setup.bash" >> ~/.${name_custom_bashrc}
fi

if ! grep -q "export ROS_MASTER_URI=http://localhost:11311" ~/.${name_custom_bashrc}; then
  echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.${name_custom_bashrc}
fi

if ! grep -q "export ROS_HOSTNAME=localhost" ~/.${name_custom_bashrc}; then
  echo "export ROS_HOSTNAME=localhost" >> ~/.${name_custom_bashrc}
fi

source /opt/ros/${name_ros_distro}/setup.bash

####################### WORKSPACE #######################

if ! [ -d ~/$name_catkinws/src ];then
  echo "---[ Creating ~/$name_catkinws workspace ]"
else
  echo "---[ Workspace ~/$name_catkinws already exists ]"
fi

mkdir -p ~/$name_catkinws/src
cd ~/$name_catkinws/src

if ! [ -f ~/$name_catkinws/src/CMakeLists.txt ]; then
  echo "---[ Initializing workspace ]"
  catkin_init_workspace
else
  echo "---[ Workspace already initialized ]"
fi

if [ ! -d ~/$name_catkinws/devel  -o  ! -d ~/$name_catkinws/build  -o  ! -f ~/$name_catkinws/devel/setup.bash ] ; then
  cd ~/$name_catkinws
  echo "---[ Compiling workspace for first time ]"
  catkin_make --force-cmake
fi

if ! grep -q "source ~/$name_catkinws/devel/setup.bash" ~/.${name_custom_bashrc}; then
  echo "source ~/$name_catkinws/devel/setup.bash" >> ~/.${name_custom_bashrc}
fi

cd ~/$name_catkinws/src

if ! [ -f ~/$name_catkinws/src/.rosinstall ]; then
  echo "---[ Initializing wstool on workspace ]"
  wstool init
fi

####################### FINISH #######################

echo "---[ Script Complete ]"

cd $start_pwd

# exec bash

return 0

