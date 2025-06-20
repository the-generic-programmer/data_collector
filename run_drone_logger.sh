#!/bin/bash
# Run the DroneLogger node from the data_collector package

# Source ROS 2 and workspace setup
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source ~/ardu_ws/src/data_collector/install/setup.bash

# Run the node using ros2 run (uses entry point from setup.py)
ros2 run data_collector my_node
