#!/bin/bash

echo "start copy yahboomcar_cartographer.launch to /opt/ros/melodic/share/cartographer_ros/launch"
sudo cp yahboomcar_cartographer.launch /opt/ros/melodic/share/cartographer_ros/launch
echo ""
echo "start copy yahboomcar.lua to /opt/ros/melodic/share/cartographer_ros/configuration_files"
sudo cp yahboomcar.lua /opt/ros/melodic/share/cartographer_ros/configuration_files
echo "finish !!!"

