# filepath: /home/autrin/Desktop/402project/sd02_joseph-hoane_1/run.sh
#!/bin/bash

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Source your workspace setup if needed
if [ -f ~/catkin_ws/devel/setup.bash ]; then
  source ~/catkin_ws/devel/setup.bash
fi

# Activate virtual environment if exists
if [ -d .venv ]; then
  source .venv/bin/activate
fi

# Install required Python packages - remove quiet flag and add verbose output
echo "Installing required Python packages..."
pip3 install python-chess stockfish
if [ $? -ne 0 ]; then
  echo "Failed to install required Python packages. Exiting."
  exit 1
fi
echo "Python packages installed successfully."

# Kill any existing ROS/Gazebo processes
echo "Cleaning up any existing ROS processes..."
killall -9 roscore rosmaster roslaunch gzserver gzclient || true
sleep 3

# Launch UR10e, MoveIt, and other required nodes
echo "Starting ROS nodes..."
roslaunch ur_gazebo ur10e_bringup.launch &
sleep 15
roslaunch ur10e_moveit_config moveit_planning_execution.launch sim:=true &
sleep 15

# Run the main application
echo "Starting chess application..."
python3 main.py