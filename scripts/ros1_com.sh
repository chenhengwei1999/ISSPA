#! /bin/bash

echo "Configuring ROS1's environment variables..."

ROS_MASTER_IPv4=$1

# ROS1
export ROS_MASTER_URI=http://$ROS_MASTER_IPv4:11311
export ROS_HOSTNAME=localhost