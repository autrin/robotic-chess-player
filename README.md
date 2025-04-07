# sd02_joseph-hoane_1

## Setup

### ROS Noetic dependencies
- Locate your catkin workspace directory (e.g. `~/catkin_ws`), we refer to this as <CATKIN WORKSPACE>

- Create a new virtual environment that inherits site packages:
```
virtualenv venv --system-site-packages
```
- You can now use the virtual environment with your IDE by pointing to `<CATKIN WORKSPACE>/src/venv/bin/python`. Or manually source it:
```
source <CATKIN WORKSPACE>/src/venv/bin/activate
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

###