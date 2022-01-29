#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

name_ros_distro="indigo"
name_catkinws="catkin_ws"
name_custom_bashrc="mybashrc"


start_pwd=$(pwd)

echo "---------------------------------------"
echo "----------- 3. SETUP KOBUKI -----------"
echo "---------------------------------------"
echo "---[ Check Ubuntu (trusty) and ROS (indigo) versions ]"
echo "---[ Install apt-get package dependencies ]"
echo "---[ Add and compile ROS packages ]"
echo "---------------------------------------"

####################### Check Ubuntu and ROS #######################

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
echo "---[ Your Ubuntu version is: $relesenum ]"

ros_distro=`rosversion -d`

if ros_distro='indigo'; then
  echo "---[ Your ROS version is: $ros_distro ]"
else
  echo "---[ ERROR: This script expects to work on ROS Indigo ]"
  return 0
fi

####################### Dependencies #######################
echo "---[ Installing apt-get package dependencies ]"
sudo apt-get install -y ssh
sudo apt-get install -y vim
sudo apt-get install -y subversion
sudo apt-get install -y cmake

sudo apt-get install -y ros-$name_ros_distro-navigation
sudo apt-get install -y ros-$name_ros_distro-gazebo*
sudo apt-get install -y ros-$name_ros_distro-turtlebot*
sudo apt-get install -y ros-$name_ros_distro-kobuki*

####################### Packages #######################
source ~/${name_catkinws}/devel/setup.bash
cd ~/${name_catkinws}/src

if ! [ -f ~/${name_catkinws}/src/upc_mrn/package.xml ]; then
  echo "---[ Adding upc_mrn package to workspace ]"
  wstool set -y -u upc_mrn --svn https://devel.iri.upc.edu/pub/labrobotica/ros/iri-ros-pkg_hydro/metapackages/iri_teaching/upc_mrn
  wstool update upc_mrn | grep upc_mrn/
else
  echo "---[ Updating upc_mrn package to last version ]"
  wstool update upc_mrn | grep upc_mrn/
fi

source ~/${name_catkinws}/devel/setup.bash

roscd upc_mrn
cd ~/${name_catkinws}
echo "---[ Compiling workspace ]"
catkin_make

if ! grep -q "source /usr/share/gazebo/setup.sh" ~/.${name_custom_bashrc}; then
  echo "source /usr/share/gazebo/setup.sh" >> ~/.${name_custom_bashrc} 
fi

if ! grep -q "export GAZEBO_" ~/.${name_custom_bashrc}; then
  echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/${name_catkinws}/src/upc_mrn/models" >> ~/.${name_custom_bashrc}
  echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:~/${name_catkinws}/src/upc_mrn/models" >> ~/.${name_custom_bashrc}
fi

if ! grep -q "export TURTLEBOT_3D_SENSOR=kinect" ~/.${name_custom_bashrc}; then
  echo "export TURTLEBOT_3D_SENSOR=kinect" >> ~/.${name_custom_bashrc} 
fi

echo "---[ Creating kobuki ftdi udev rule ]"
rosrun kobuki_ftdi create_udev_rules

####################### FINISH #######################

echo "---[ Script Complete ]"

cd $start_pwd

# exec bash

return 0

