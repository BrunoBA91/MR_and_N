#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

name_ros_distro="indigo"
#
name_custom_bashrc="mybashrc"


start_pwd=$(pwd)

echo "---------------------------------------"
echo "-------- 1. ROS INSTALL SCRIPT  -------"
echo "---------------------------------------"
echo "---[ For Ubuntu Trusty 14.04 ]"
echo "---[ Install ROS version: $name_ros_distro ]"
echo "---[ Setups config in:    ~/.$name_custom_bashrc ]"
echo "---------------------------------------"

####################### ROS INSTALL #######################

version=`lsb_release -sc`

echo "---[ Checking the ubuntu version ]"
case $version in
  "trusty")
  ;;
  *)
    echo "---[ ERROR: This script expects to work on ubuntu trusty(14.04) ]"
    return 0
esac

relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
echo "---[ Your ubuntu version is: $relesenum ]"

echo "---[ Update & upgrade apt-get packages ]"
sudo apt-get update
sudo apt-get -y upgrade

echo "---[ Add the ROS repository ]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi

echo "---[ Download the ROS keys ]"
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116

echo "---[ Update & upgrade apt-get packages ]"
sudo apt-get update
sudo apt-get -y upgrade

echo "---[ Installing ROS desktop full ]"
sudo apt-get install -y ros-$name_ros_distro-desktop-full

#echo "---[ Installing other ROS packages ]"
#sudo apt-get install -y ros-$name_ros_distro-navigation

echo "---[ rosdep init and python-rosinstall ]"
sudo rosdep init
rosdep update
. /opt/ros/$name_ros_distro/setup.sh
sudo apt-get install -y python-rosinstall

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

####################### FINISH #######################

echo "---[ Script Complete ]"

cd $start_pwd

# exec bash

return 0

