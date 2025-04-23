#!/bin/bash
# run.sh - Clean up existing ROS processes, start a fresh roscore,
# then launch the UR10e simulator, MoveIt nodes, and your pick-and-drop node.

# Exit immediately if a command fails
set -e

echo "Cleaning up any existing ROS nodes and processes..."
# Kill all ROS nodes and lingering processes
rosnode kill -a || true
killall -9 roscore rosmaster roslaunch gzserver gzclient || true
sleep 3

echo "Starting a fresh roscore..."
roscore &
CORE_PID=$!
# Wait until the ROS master is available.
echo "Waiting for ROS master..."
until rostopic list >/src/null 2>&1; do
    sleep 1
done
echo "ROS master is available."

# Source the global ROS setup
source /opt/ros/noetic/setup.bash

# Source the catkin workspace setup (adjust path if needed)
if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
else
    echo "Warning: Catkin workspace setup not found at ~/catkin_ws/devel/setup.bash"
fi

# Activate your virtual environment if it exists
if [ -d .venv ]; then
    source .venv/bin/activate
else
    echo "Warning: Virtual environment (.venv) not found."
fi

# Wait a little for the new roscore to propagate
sleep 2

# Delete any existing robot model in Gazebo (if any)
echo "Deleting existing robot model in Gazebo (if any)..."
rosservice call /gazebo/delete_model "model_name: 'robot'" || true

# Launch the UR10e Gazebo simulator in the background
echo "Launching UR10e simulator..."
roslaunch ur_gazebo ur10e_bringup.launch &
SIM_PID=$!

# Launch the MoveIt planning and execution node (simulation mode) in the background
echo "Launching MoveIt planning execution..."
roslaunch ur10e_moveit_config moveit_planning_execution.launch sim:=true &
MOVEIT_PID=$!

# Wait for the simulator and MoveIt nodes to initialize.
echo "Waiting for simulator and MoveIt initialization..."
sleep 20

# Wait until the move_group action server becomes available.
echo "Waiting for move_group action server..."
until rostopic list | grep -q "/move_group/goal"; do
    sleep 1
done
echo "move_group action server is now available."

# Check if the robot_description is loaded.
if ! rosparam get /robot_description >/src/null 2>&1; then
    echo "Error: 'robot_description' parameter is not loaded. Please check your launch files."
    kill -SIGINT $SIM_PID $MOVEIT_PID $CORE_PID
    exit 1
fi

# Run your pick-and-drop node using rosrun.
# Replace 'sd02_joseph-hoane_1' with the actual name of your ROS package.
echo "Running pick-and-drop node..."
rosrun sd02_joseph-hoane_1 ur10e_pick_and_drop.py

# After the node finishes, gracefully shut down the background processes.
echo "Shutting down simulator, MoveIt, and roscore gracefully..."
kill -SIGINT $SIM_PID $MOVEIT_PID $CORE_PID
wait $SIM_PID $MOVEIT_PID $CORE_PID

# Ensure that any remaining Gazebo processes are killed.
echo "Killing any remaining Gazebo processes..."
killall -9 gzserver gzclient || true

echo "Run complete."
