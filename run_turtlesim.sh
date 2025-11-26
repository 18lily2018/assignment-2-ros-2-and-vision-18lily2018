#!/bin/bash
# Launcher for turtlesim (Terminal C)

# Load ROS2 + workspace
source ~/.ros2_auto_setup.sh

# Go to workspace (not strictly needed, but consistent)
cd ~/ros2_ws_clean

# Run turtlesim GUI
ros2 run turtlesim turtlesim_node



