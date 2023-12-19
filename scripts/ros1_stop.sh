#! /bin/bash

# Terminate all ROS nodes
rosnode kill -a

# Alternatively, you can use the following command with ps to kill all ROS nodes:
ps aux | grep -i 'ros' | awk '{print $2}' | xargs kill -9