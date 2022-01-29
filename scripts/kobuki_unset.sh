#!/bin/bash

## EDITABLE NAMES
BASHRCNAME="mybashrc"
##

sed --in-place '/ROS_HOSTNAME=10.42.0.1/d' ~/.${BASHRCNAME}

unset ROS_IP
unset ROS_HOSTNAME
unset ROS_MASTER_URI

source ~/.${BASHRCNAME}
