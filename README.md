# sd02_joseph-hoane_1

### Directory setup:
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ ls -a
	.  ..  build  .catkin_workspace  devel  src  .vscode
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ ls -a src/
	.   CMakeLists.txt  robotiq              universal_robot              ur_msgs
	..  isu_rel         sd02_joseph-hoane_1  Universal_Robots_ROS_Driver
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ 

### To activate the workspace and run:
	
	cd /home/autrin/Desktop/402project/project/catkin_ws
	
	catkin_make
	
	source ~/Desktop/402project/project/catkin_ws/devel/setup.bash
	
	# For simulation mode (default)
		roslaunch sd02_joseph-hoane_1 chess_robot.launch
		OR
		bash ~/Desktop/402project/project/catkin_ws/src/sd02_joseph-hoane_1/launch_simulation.sh
	
	# When connecting to the actual robot
		roslaunch sd02_joseph-hoane_1 chess_robot.launch sim:=false robot_ip:=192.168.1.100
	
	Manually test chess moves without the vision system:
		roslaunch sd02_joseph-hoane_1 chess_robot.launch _chess_robot:=test
		OR
		rosrun sd02_joseph-hoane_1 run_chess_robot.py test



## Setup

### ROS Noetic python dependencies

*At this point, if you are using an external catkin workspace, skip to the next section*

Go inside catwin_ws, and create a new python virtual environment that inherits site packages:

```
virtualenv venv --system-site-packages
```

You can now use the virtual environment with your IDE by pointing to `catkin_ws/src/venv/bin/python`. Or manually source
it:

```
source catkin_ws/src/venv/bin/activate
```

Alternatively, you can create a symlink in the project root that points to the catkin virtual environment after creating
it, and use it from project root:

```
ln -s catkin_ws/src/venv ./venv
```

If using PyCharm, optionally also add `opt/ros/noetic/lib/python3/dist-packages` as another content root in Project
Structure.
Also, mark `catkin_ws`, `venv`, and `dist-packages` as excluded by right-clicking on them, then find "Mark Directory
as > Excluded".

### Add robot descriptor for UR10e + Robotiq Hand-E gripper to workspace

`jh_ur10e_hande` is a our custom package that describes a system that consists of a Robotiq Hand-E gripped attached as
an end-effector to a Universal Robotics UR10e arm.
It can be found in the project root and is symbolically linked to `catkin_ws/src/jh_ur10e_hande`. Make sure this
symbolic link exists, if not, then create it with:

```
ln -s /path/to/project_root/jh_ur10e_hande/ catkin_ws/src/jh_ur10e_hande
```

Note that the source path must be absolute in this case.

The Hand-E gripper uses a GripperActionController, which needs to be installed:

```
sudo apt install ros-noetic-gripper-action-controller
```

Then remember to load the environment before usage:

```
cd catkin_ws
catkin_make
source devel/setup.bash
```

## Setup ONLY IF USING external catkin workspace

### ROS Noetic python dependencies

Locate your catkin workspace directory (e.g. `~/catkin_ws`), we refer to this as \<CATKIN WORKSPACE\>

Create a new virtual environment that inherits site packages:

```
virtualenv venv --system-site-packages
```

You can now use the virtual environment with your IDE by pointing to `<CATKIN WORKSPACE>/src/venv/bin/python`. Or
manually source it:

```
source <CATKIN WORKSPACE>/src/venv/bin/activate
```

### Add and install Robotiq Hand-E gripper descriptors

We will use descriptors from [https://github.com/cambel/robotiq.git].

```
cd <CATKIN_WORKSPACE>
git clone https://github.com/cambel/robotiq.git
rosdep update
rosdep install --from-paths robotiq --ignore-src -y
```

Remember to reload the environment:

```
cd <CATKIN_WORKSPACE>
catkin_make
source devel/setup.bash
```

### Add robot descriptor for UR10e + Robotiq Hand-E gripper to workspace:

Create a symlink from the `jh_ur10e_hande` package in the project to your catkin workspace:

```
ln -s <PATH TO THIS REPOSITORY>/jh_ur10e_hande <CATKIN WORKSPACE>/SRC/jh_ur10e_hande
```

Then rebuild the workspace:

```
cd <CATKIN WORKSPACE>
catkin_make
```

### Install `jh1` as a python library

In your virtual environment, run:

```
pip install -e ./src

```

## Usage

### Launch

- Gazebo: robot with gripper and joint controllers

```
# Base command
roslaunch jh_ur10e_hande gazebo_bringup.launch

# Or launch inside a screen task
screen -S rl -dm bash -c "roslaunch jh_ur10e_hande gazebo_bringup.launch"
```

- MoveIt: with overridden robot description and SRDF

```
# Base command
roslaunch jh_ur10e_hande moveit_planning_execution.launch

# Or launch inside a screen task
screen -S rgz -dm bash -c "roslaunch jh_ur10e_hande moveit_planning_execution.launch"
```

- *Note: if running on WSL, run this command first to allow `screen` to work*

```
sudo /etc/init.d/screen-cleanup start
```

