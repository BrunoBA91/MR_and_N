#!/bin/bash
# This script creates a ROS workspace
#######################Configurable#######################
BASHRCNAME="mybashrc"
WSNAME="catkin_ws"
SUDO=false
#######################Logging#######################
#REDIRECTION="/dev/null"
DEST=${BASH_SOURCE[0]}
FILE="${DEST##*/}"
FILENAME="log_${FILE%.*}"
LOGDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
#LOGDIR="${HOME}"
REDIRECTION="${LOGDIR}/${FILENAME}.log"
> ${REDIRECTION}

ROSVERSION1="indigo"
ROSVERSION2="kinetic"
##OpenCV Kinetic fix
# sudo ln -s /opt/ros/kinetic/lib/libopencv_core3.so /opt/ros/kinetic/lib/libopencv_core3.so.3.2


MYPWD=$(pwd)


#ROSVERSION=$(ls /opt/ros | head -n 1)
#ROSVERSION="$(rosversion -d)"
if [ -f /opt/ros/${ROSVERSION1}/setup.bash ]; then
  ROSVERSION=${ROSVERSION1}
  echo " --- Found installed ROS ${ROSVERSION}"
elif [ -f /opt/ros/${ROSVERSION2}/setup.bash ]; then
  ROSVERSION=${ROSVERSION2}
  echo " --- Found installed ROS ${ROSVERSION}"
else
  echo "ERROR: no ${ROSVERSION1} or ${ROSVERSION2} ROS version installed found. Exiting"
  return
fi

#################################################################
### MYBASHRC

if ! [ -f ~/.${BASHRCNAME} ];then
  echo "### UPC_MRN: ROS Setup" >> ~/.${BASHRCNAME}
  echo "### Sourced by ~/.bashrc" >> ~/.${BASHRCNAME}
fi

if ! grep -q "source /opt/ros/${ROSVERSION}/setup.bash" ~/.${BASHRCNAME}; then
  echo "source /opt/ros/${ROSVERSION}/setup.bash" >> ~/.${BASHRCNAME}
fi

if ! grep -q "source ~/${WSNAME}/devel/setup.bash" ~/.${BASHRCNAME}; then
  echo "source ~/${WSNAME}/devel/setup.bash" >> ~/.${BASHRCNAME}
fi

#################################################################
### BASHRC

if ! grep -q "source ~/.${BASHRCNAME}" ~/.bashrc; then
  echo " --- Adding ~/.${BASHRCNAME} source to ~/.bashrc..."
  echo "### UPC_MRN: ROS Setup " >> ~/.bashrc
  echo "if [ -f ~/.${BASHRCNAME} ]; then" >> ~/.bashrc
  echo "  source ~/.${BASHRCNAME}" >> ~/.bashrc
  echo "else" >> ~/.bashrc
  echo "echo \"  Error: couldn't source ~/.mybashrc. File not found.\"" >> ~/.bashrc
  echo "fi" >> ~/.bashrc
  echo "### ### " >> ~/.bashrc
fi

#################################################################
### WORKSPACE

source /opt/ros/${ROSVERSION}/setup.bash

if ! [ -d ~/${WSNAME}/src ];then
  echo " --- Creating ~/${WSNAME} workspace ..."
  mkdir -p ~/${WSNAME}/src
fi


if ! [ -f ~/${WSNAME}/src/CMakeLists.txt ]; then
  echo " --- Initializing ~/${WSNAME} workspace..."
  cd ~/${WSNAME}/src
  catkin_init_workspace &>> ${REDIRECTION}
fi

if [ ! -d ~/${WSNAME}/devel  -o  ! -d ~/${WSNAME}/build  -o  ! -f ~/${WSNAME}/devel/setup.bash ] ; then
  cd ~/${WSNAME}
  echo " --- Compiling ~/${WSNAME} workspace for first time..."
  catkin_make --force-cmake &>> ${REDIRECTION}
fi

source ~/${WSNAME}/devel/setup.bash
cd ~/${WSNAME}/src

if ! [ -f ~/${WSNAME}/src/.rosinstall ]; then
  echo " --- Initializing wstool on '${WSNAME}' workspace..."
  wstool init &>> ${REDIRECTION}
fi

#################################################################
### ADD PACKAGES

function add_apt_pkg #ARGS: pkg
{
  PKG=$1
  echo " --- Installing apt-get ros-${ROSVERSION}-${PKG} package"
  sudo apt-get install -y ros-${ROSVERSION}-${PKG} &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
}

function add_source_pkg #ARGS: pkg type url
{
  PKG=$1
  TYPE=$2
  URL=$3
  
  cd ~/${WSNAME}/src
  #if ! rospack -q find ${PKG} &>> ${REDIRECTION} || rosstack -q find ${PKG} &>> ${REDIRECTION}; then
  if ! [ -d ~/${WSNAME}/src/${PKG} ]; then
    echo " --- Downloading ROS ${PKG} package from ${URL}"
    wstool set ${PKG} -y --${TYPE} ${URL} &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
  fi
  echo " --- Updating ROS ${PKG} package from ${URL}"
  wstool update ${PKG} &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
  source ~/${WSNAME}/devel/setup.bash
  roscd ${PKG}
  echo " --- Compiling ${PKG} package"
  cd ~/${WSNAME}
  catkin_make --only-pkg-with-deps ${PKG} &>> ${REDIRECTION} || echo "###ERROR: ${PKG} compilation failed, for more info check log file: $LOGDIR/$FILENAME.log"
  # catkin_make -DCATKIN_WHITELIST_PACKAGES="${PKG}" &>> ${REDIRECTION}
}

add_source_pkg upc_mrn svn https://devel.iri.upc.edu/pub/labrobotica/ros/iri-ros-pkg_hydro/metapackages/iri_teaching/upc_mrn

if ! grep -q "source /usr/share/gazebo/setup.sh" ~/.${BASHRCNAME}; then
  echo " --- Adding Gazebo paths to ~/.${BASHRCNAME}..."
  echo "source /usr/share/gazebo/setup.sh" >> ~/.${BASHRCNAME} 
fi

if ! grep -q "export GAZEBO_" ~/.${BASHRCNAME}; then
  echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/${WSNAME}/src/upc_mrn/models" >> ~/.${BASHRCNAME}
  #echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/${WSNAME}/src/upc_mrn/models" >> ~/.${BASHRCNAME}
fi

if ! grep -q "export TURTLEBOT_3D_SENSOR=kinect" ~/.${BASHRCNAME}; then
  echo "export TURTLEBOT_3D_SENSOR=kinect" >> ~/.${BASHRCNAME} 
fi

source ~/.${BASHRCNAME}

if [ "$SUDO" = true ] ; then
  add_apt_pkg navigation
  add_apt_pkg gmapping
  add_apt_pkg gazebo*
  add_apt_pkg turtlebot*
  add_apt_pkg kobuki*
  echo " --- Creating kobuki ftdi udev rule"
  rosrun kobuki_ftdi create_udev_rules &>> ${REDIRECTION}
fi

cd $MYPWD
echo " --- Done."