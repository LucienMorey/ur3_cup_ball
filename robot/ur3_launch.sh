#!/bin/bash

source ~/.bashrc
source "$HOME"/.config/robot_config.cfg

echo "Checking ROS node status ..."
ROS_NODE_CHECK=/robot_state_publisher
ROS_NODE_LIST="$(rosnode list)"

if [[ $ROS_NODE_LIST == *"$ROS_NODE_CHECK"* ]]; then
  echo "UR DRIVER ROS NODE IS RUNNING - SHUTTING DOWN TO OVERRIDE"
  rosnode kill --all
fi

roslaunch ur_robot_driver ur3_bringup.launch robot_ip:="$UR3_IP" kinematics_config:="$UR3_CALIB_FILE"
