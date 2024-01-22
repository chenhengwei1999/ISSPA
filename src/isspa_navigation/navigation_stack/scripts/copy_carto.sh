#!/bin/bash

echo "start copy pavs_cartographer.launch to ~/share/cartographer_ros/launch"
sudo cp pavs_cartographer.launch ~/share/cartographer_ros/launch
echo ""
echo "start copy pavs.lua to ~/share/cartographer_ros/configuration_files"
sudo cp pavs.lua ~/share/cartographer_ros/configuration_files
echo "finish !!!"

