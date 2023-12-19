#! /bin/bash

echo "Configuring ROS1's environment variables..."

# ROS1
export ROS_MASTER_URI=http://$1:11311
export ROS_HOSTNAME=$(ip -4 addr show wlo1 | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1)

echo "ROS_MASTER_URI is set to $ROS_MASTER_URI"
echo "ROS_HOSTNAME is set to $ROS_HOSTNAME"
