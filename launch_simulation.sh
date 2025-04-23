#!/bin/bash

# Kill any existing ROS processes first
killall -9 roscore rosrun roslaunch || true
sleep 1

# Start roscore in background
roscore &
sleep 2

# Load robot description - correct path
roslaunch ur_description load_ur10e.launch &
sleep 2

# Start simulator
rosrun isu_rel simulator.py &
sleep 2

# Start RViz with configuration
# First check if the file exists
if [ -f "$(rospack find ur_description)/rviz/ur10e.rviz" ]; then
    RVIZ_CONFIG="$(rospack find ur_description)/rviz/ur10e.rviz"
elif [ -f "$(rospack find ur_description)/rviz/ur10e_default.rviz" ]; then
    RVIZ_CONFIG="$(rospack find ur_description)/rviz/ur10e_default.rviz"
else
    # Use default config if specific config doesn't exist
    RVIZ_CONFIG=""
fi

# Launch RViz with proper config
if [ -n "$RVIZ_CONFIG" ]; then
    rosrun rviz rviz -d $RVIZ_CONFIG
else
    rosrun rviz rviz
fi

# Kill all background processes when script exits
trap "killall -9 roscore rosrun roslaunch" EXIT