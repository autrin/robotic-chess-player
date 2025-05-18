# Robotic Chess Player

This repository contains a complete ROS workspace for a robotic chess player using a UR10e robot arm with a Robotiq gripper.

## Repository Structure

This is a complete Catkin workspace that includes:
- Our main package (`sd02_joseph-hoane_1`)
- All necessary drivers:
  - `robotiq` - Robotiq gripper drivers
  - `universal_robot` - Universal Robots description and configuration
  - `ur_msgs` - Universal Robots message definitions
  - `Universal_Robots_ROS_Driver` - Main UR driver
- `isu_rel` - Additionaly for testing purposes

## Quick Start

```bash
# 1. Clone the repository
git clone https://github.com/autrin/robotic-chess-player.git
cd robotic-chess-player

# 2. Build the workspace
catkin_make

# 3. Source the setup file
source devel/setup.bash
```

## Running the Robot

### With Real Hardware

1. Launch the UR robot driver
   ```bash
   roslaunch ur_robot_driver ur10e_griper_bringup.launch
   ```

2. Access external control on the teach pendant

3. Launch the Robotiq gripper driver (if not already initialized)
   ```bash
   roslaunch robotiq_2f_gripper_control robotiq_action_server.launch
   ```

4. Run the main program
   ```bash
   rosrun sd02_joseph-hoane_1 main test
   # OR for production mode
   rosrun sd02_joseph-hoane_1 main
   ```

### With Simulator

1. Start ROS core
   ```bash
   roscore
   ```

2. Launch the simulator
   ```bash
   rosrun sd02_joseph-hoane_1 simulator.py
   ```

3. Run the main program in test mode
   ```bash
   rosrun sd02_joseph-hoane_1 main.py test
   ```


## Development

### Install `jh1` as a Python Library

For development, you can install the Python package in development mode:

```bash
pip install -e ./src
```

## Documentation

For more information, see the [Final Report](Document/SD02-Final-Report.pdf)


