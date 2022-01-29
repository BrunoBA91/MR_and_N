#!/bin/bash

## EDITABLE NAMES
BASHRCNAME="mybashrc"
##

echo "export ROS_HOSTNAME=10.42.0.1 && export ROS_IP=10.42.0.1 && export ROS_MASTER_URI=http://10.42.0.1:11311" >> ~/.${BASHRCNAME}

source ~/.${BASHRCNAME}
