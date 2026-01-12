#!/bin/bash
# Helper script to run x5_driver_node with sudo
# Use this if you haven't set up udev rules

# Find the workspace setup file
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    WS_SETUP="$HOME/catkin_ws/devel/setup.bash"
elif [ -f "$HOME/slambox_ws/devel/setup.bash" ]; then
    WS_SETUP="$HOME/slambox_ws/devel/setup.bash"
else
    echo "Error: Could not find catkin workspace setup.bash"
    echo "Please edit this script to set WS_SETUP to your workspace path"
    exit 1
fi

ROS_SETUP="/opt/ros/noetic/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS setup not found at $ROS_SETUP"
    exit 1
fi

echo "Running x5_driver_node with sudo..."
echo "ROS setup: $ROS_SETUP"
echo "Workspace: $WS_SETUP"
echo ""

# Pass through ROS environment variables
sudo -E bash -c "source $ROS_SETUP && source $WS_SETUP && roslaunch x5_ros_driver x5_test.launch $*"
