CST8504 Assignment 2 - ROS2 Vision
Student: 18lily2018

Packages:
  - aisd_vision: image_publisher and hands nodes
  - aisd_motion: move node

Basic usage (after colcon build and sourcing install/local_setup.bash):

  # Start turtlesim (VM) with cmd_vel remap
  ros2 run turtlesim turtlesim_node --ros-args --remap /turtle1/cmd_vel:=cmd_vel &

  # Start vision and motion nodes
  ros2 run aisd_vision image_publisher &
  ros2 run aisd_motion move &
  ros2 run aisd_vision hands

See assignment document for full instructions.
