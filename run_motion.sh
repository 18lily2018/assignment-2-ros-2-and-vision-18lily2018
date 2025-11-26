#!/bin/bash
# Launcher for motion node (Terminal A)

# Load ROS2 + workspace
source ~/.ros2_auto_setup.sh

# Go to workspace (just in case)
A

cd ~/ros2_ws_clean

# Run motion node
ros2 run aisd_motion move

