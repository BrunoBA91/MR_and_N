#!/bin/bash
# This script installs ROS Indigo
#######################Configurable#######################
TOOLS="subversion ssh vim"
BASHRCNAME="mybashrc"
ROSVERSION="indigo" #this script only works for indigo

#######################Logging#######################
#REDIRECTION="/dev/null"
DEST=${BASH_SOURCE[0]}
FILE="${DEST##*/}"
FILENAME="log_${FILE%.*}"
#LOGDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOGDIR="${HOME}/Desktop"
REDIRECTION="${LOGDIR}/${FILENAME}.log"
> ${REDIRECTION}

MYPWD=$(pwd)

###################################################

echo " --- Checking the ubuntu version"
releasenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
echo " --- Your ubuntu version is: $releasenum"

version=`lsb_release -sc`
case $version in
  "trusty")
  ;;
  *)
    echo "### ERROR: This script expects to work on ubuntu trusty(14.04)"
    return 0
esac

###################################################

echo " --- Installing tools: ${TOOLS}"
sudo apt-get install -y ${TOOLS} &>> ${REDIRECTION}

###################################################

echo " --- Checking presence of ROS ${ROSVERSION}"
if ! [ -f /opt/ros/${ROSVERSION}/setup.bash ]; then
  echo " --- Installing/updating ROS ${ROSVERSION}"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &>> ${REDIRECTION}
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116 &>> ${REDIRECTION}
  sudo apt-get update &>> ${REDIRECTION}
  sudo apt-get install -y ros-${ROSVERSION}-desktop-full &>> ${REDIRECTION}
  sudo rosdep init &>> ${REDIRECTION}
  rosdep update &>> ${REDIRECTION}
  sudo apt-get install -y python-rosinstall &>> ${REDIRECTION}
else
  echo " --- Found already installed ROS ${ROSVERSION}"
fi

if ! [ -f /opt/ros/${ROSVERSION}/setup.bash ]; then
  echo "###ERROR: ROS ${ROSVERSION} version installed not found. Exiting"
  return
fi

if ! grep -q "source ~/.${BASHRCNAME}" ~/.bashrc; then
  echo " --- Adding ~/.${BASHRCNAME} source to ~/.bashrc..."
  echo "# ~/.${BASHRCNAME}: ROS Setup " >> ~/.bashrc
  echo "if [ -f ~/.${BASHRCNAME} ]; then" >> ~/.bashrc
  echo "source ~/.${BASHRCNAME}" >> ~/.bashrc
  echo "fi" >> ~/.bashrc
fi

if ! [ -f ~/.${BASHRCNAME} ];then
  echo "# ~/.${BASHRCNAME} with ROS Setup" >> ~/.${BASHRCNAME}
  echo "# Sourced by ~/.bashrc" >> ~/.${BASHRCNAME}
fi

if ! grep -q "source /opt/ros/${ROSVERSION}/setup.bash" ~/.${BASHRCNAME}; then
  echo " --- Adding ROS ${ROSVERSION} source to ~/.${BASHRCNAME}..."
  echo "source /opt/ros/${ROSVERSION}/setup.bash" >> ~/.${BASHRCNAME}
fi

source /opt/ros/${ROSVERSION}/setup.bash

###################################################
cd $MYPWD
echo " --- Done."