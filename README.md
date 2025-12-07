CST8504 – Assignment 3
Student: <YPWANG>, <STUTDEN NUMBER>
Repository: https://github.com/18lily2018/assignment-2-ros-2-and-vision-18lily2018

-------------------------------------------------------
1. Project Overview
-------------------------------------------------------
This workspace extends Assignment 2 (motion, vision, and aisd_msgs) by adding
two new ROS 2 packages:

  • aisd_hearing  – audio / hearing node
  • aisd_speaking – speech / speaking node

All packages are Python-based ROS 2 packages that build with colcon under
ROS 2 Humble using the standard layout (setup.py, package.xml, setup.cfg,
resource/, and a Python package directory with __init__.py and node scripts).

The following packages are included in this submission:

  • aisd_motion     – motion node(s) from Assignment 2
  • aisd_vision     – vision / hand tracking from Assignment 2
  • aisd_msgs       – custom message definitions used by the nodes
  • aisd_hearing    – new hearing node(s) for Assignment 3
  • aisd_speaking   – new speaking node(s) for Assignment 3

-------------------------------------------------------
2. How to Build the Workspace
-------------------------------------------------------
Assume the workspace root is: ~/ros2_ws_clean

1) Copy the src/ subfolders from this submission into the workspace:
     ~/ros2_ws_clean/src/aisd_motion
     ~/ros2_ws_clean/src/aisd_vision
     ~/ros2_ws_clean/src/aisd_msgs
     ~/ros2_ws_clean/src/aisd_hearing
     ~/ros2_ws_clean/src/aisd_speaking

2) From the workspace root:

     cd ~/ros2_ws_clean
     colcon build

3) Source the workspace:

     source /opt/ros/humble/setup.bash
     source ~/ros2_ws_clean/install/setup.bash

-------------------------------------------------------
3. How to Run the Demo Nodes
-------------------------------------------------------

Basic setup for all terminals (after build):

  source /opt/ros/humble/setup.bash
  source ~/ros2_ws_clean/install/setup.bash

Example demo sequence:

1) Start motion / vision (from Assignment 2):

   # Terminal 1 – motion
   ros2 run aisd_motion move

   # Terminal 2 – vision / hands
   ros2 run aisd_vision hands

2) Start hearing node (Assignment 3):

   # Terminal 3 – hearing
   ros2 run aisd_hearing <hearing_node_name>

3) Start speaking node (Assignment 3):

   # Terminal 4 – speaking
   ros2 run aisd_speaking <speaking_node_name>

Replace <hearing_node_name> and <speaking_node_name> with the actual
Python entry point names defined in setup.py.

-------------------------------------------------------
4. Package Summary
-------------------------------------------------------

aisd_motion:
  • Provides motion-related nodes using geometry_msgs/Twist etc.
  • Reused from Assignment 2.

aisd_vision:
  • Provides vision / hand tracking nodes and publishes aisd_msgs::Hand
    (or similar) messages.
  • Reused from Assignment 2.

aisd_msgs:
  • Contains the custom message definitions used by the vision and motion
    nodes (and reused by hearing/speaking where appropriate).

aisd_hearing:
  • New package for Assignment 3.
  • Implements the "hearing" node(s). These subscribe to the relevant
    topics (for example audio or command topics) and publish events or
    messages that other nodes can consume.
  • Implemented in Python with standard ROS 2 rclpy node structure.

aisd_speaking:
  • New package for Assignment 3.
  • Implements the "speaking" node(s). These subscribe to command or
    text topics and produce synthesized speech or message output.
  • Implemented in Python with standard ROS 2 rclpy node structure.

-------------------------------------------------------
5. Notes / Known Issues
-------------------------------------------------------

  • The build has been tested in the school ROS2 VM environment.
  • Folders such as build/, install/, log/, and recordings/ are deliberately
    excluded from the submission and from GitHub; they are regenerated
    by colcon build and during runtime.
  • If there are any problems running the demo, please contact me and I
    can demonstrate on my VM where the packages build and run successfully.
