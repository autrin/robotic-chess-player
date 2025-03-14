#!/bin/bash
# run.sh - Script to set up ROS environment and run the UR10e pick-and-drop node

# Exit immediately if a command exits with a non-zero status
set -e

# Source global ROS setup
source /opt/ros/noetic/setup.bash

# Source your catkin workspace setup (adjust path if your workspace is elsewhere)
if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
else
    echo "Warning: Catkin workspace setup not found at ~/catkin_ws/devel/setup.bash"
fi

# Activate your Python virtual environment (adjust path if necessary)
if [ -d .venv ]; then
    source .venv/bin/activate
else
    echo "Warning: Virtual environment not found in .venv"
fi

# Run the UR10e pick-and-drop script from the practice folder
# python practice/ur10e_pick_and_drop.py
