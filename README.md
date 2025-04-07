# sd02_joseph-hoane_1

## Setup

### ROS Noetic dependencies
Locate your catkin workspace directory (e.g. `~/catkin_ws`), we refer to this as <CATKIN WORKSPACE>

Create a new virtual environment that inherits site packages:
```
virtualenv venv --system-site-packages
```

You can now use the virtual environment with your IDE by pointing to `<CATKIN WORKSPACE>/src/venv/bin/python`. Or manually source it:
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
- Create a symlink from the `jh_ur10e_hande` package in the project to your catkin workspace:
```
ln -s <PATH TO THIS REPOSITORY>/jh_ur10e_hande <CATKIN WORKSPACE>/SRC/jh_ur10e_hande
```

- Then rebuild the workspace:
```
cd <CATKIN WORKSPACE>
catkin_make
```


## Usage

### Launch
- Gazebo: robot with gripper and joint controllers
```
screen -S rl -dm bash -c "roslaunch jh_ur10e_hande gazebo_bringup.launch"
```

- MoveIt: with overridden robot description and SRDF
```
screen -S rgz -dm bash -c "roslaunch jh_ur10e_hande moveit_planning_execution.launch"
``

- *Note: if running on WSL, run this command first to allow `screen` to work*
```
sudo /etc/init.d/screen-cleanup start
```

