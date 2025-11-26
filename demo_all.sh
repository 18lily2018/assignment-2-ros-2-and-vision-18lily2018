#!/bin/bash
# ALL-IN-ONE demo launcher for CST8504 Assignment 2
# Starts: turtlesim, motion, hands, and camera publisher

set -e

# 1. Load ROS2 + your workspace (we created this earlier)
source ~/.ros2_auto_setup.sh

# 2. Go to workspace (just in case)
cd ~/ros2_ws_clean

echo "==============================="
echo " Starting ALL NODES for DEMO"
echo " - turtlesim"
echo " - aisd_motion move"
echo " - aisd_vision hands"
echo " - aisd_vision image_publisher"
echo "==============================="

# 3. Start turtlesim (needs VcXsrv running on Windows)
echo "[1/4] Starting turtlesim_node..."
ros2 run turtlesim turtlesim_node &
PID_TURTLE=$!

# Give turtlesim a moment to open the window
sleep 3

# 4. Start motion node
echo "[2/4] Starting aisd_motion move..."
ros2 run aisd_motion move &
PID_MOTION=$!

# 5. Start hands node
echo "[3/4] Starting aisd_vision hands..."
ros2 run aisd_vision hands &
PID_HANDS=$!

# 6. Start camera publisher
echo "[4/4] Starting aisd_vision image_publisher..."
ros2 run aisd_vision image_publisher &
PID_CAM=$!

echo "--------------------------------"
echo "All nodes started!"
echo "  turtlesim_node PID : $PID_TURTLE"
echo "  move PID           : $PID_MOTION"
echo "  hands PID          : $PID_HANDS"
echo "  image_publisher PID: $PID_CAM"
echo "--------------------------------"
echo "Press ENTER to stop everything..."
read _

echo "Stopping all nodes..."
kill $PID_CAM $PID_HANDS $PID_MOTION $PID_TURTLE 2>/dev/null || true
wait 2>/dev/null || true
echo "All nodes stopped. Demo finished."


