#!/bin/bash
# run.sh - Launch the UR10e simulator and your pick-and-drop node

# Exit immediately if a command fails
set -e

# Source the global ROS setup
source /opt/ros/noetic/setup.bash

# Source the catkin workspace setup (adjust path if needed)
if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
else
    echo "Warning: Catkin workspace setup not found at ~/catkin_ws/devel/setup.bash"
fi

# Activate your virtual environment
if [ -d .venv ]; then
    source .venv/bin/activate
else
    echo "Warning: Virtual environment (.venv) not found."
fi

# Optionally, start roscore if it's not already running
# roslaunch automatically starts roscore if needed, so this may not be required.
# roscore &

# Launch the UR10e Gazebo simulator in the background
echo "Launching UR10e simulator..."
roslaunch ur_gazebo ur10e_bringup.launch &
SIM_PID=$!

# Launch the MoveIt planning and execution node (simulation mode)
echo "Launching MoveIt planning execution..."
roslaunch ur10e_moveit_config moveit_planning_execution.launch sim:=true &
MOVEIT_PID=$!

# Wait for the simulation and MoveIt to initialize (adjust sleep duration if needed)
echo "Waiting for simulator initialization..."
sleep 15

# Run your pick-and-drop node using rosrun.
# Replace <your_package_name> with the actual name of your package.
echo "Running pick-and-drop node..."
rosrun sd02_joseph-hoane_1 ur10e_pick_and_drop.py

# After the node finishes, kill the background processes
echo "Shutting down simulator and MoveIt nodes..."
kill $SIM_PID $MOVEIT_PID

echo "Run complete."
