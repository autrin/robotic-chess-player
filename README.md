# sd02_joseph-hoane_1

### Directory setup:
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ ls -a
	.  ..  build  .catkin_workspace  devel  src  .vscode
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ ls -a src/
	.   CMakeLists.txt  robotiq              universal_robot              ur_msgs
	..  isu_rel         sd02_joseph-hoane_1  Universal_Robots_ROS_Driver
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ 
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ ls -a src/sd02_joseph-hoane_1/
	.   build           Document  .gitattributes  jh_ur10e_hande  launch_simulation.sh  meeting-notes.md  paths.csv  progress.md  resources    run.sh  stockfish
	..  CMakeLists.txt  .git      .gitignore      launch          main.py               package.xml       practice   README.md    runRobot.sh  src
	autrin@autrin-rog-zephyrus-g14-ga403ui:~/Desktop/402project/project/catkin_ws$ 

### To activate the workspace and run:
	
	cd /home/autrin/Desktop/402project/project/catkin_ws
	
	catkin_make
	
	• Run full (vision) in simulation (default):
	roslaunch sd02_joseph‑hoane_1 chess_robot.launch

	• Run full (vision) with the real robot:
	roslaunch sd02_joseph‑hoane_1 chess_robot.launch sim:=false robot_ip:=192.168.1.100

	• Run test mode (no vision) in simulation:
	roslaunch sd02_joseph‑hoane_1 chess_robot.launch chess_robot:=test

	• Run test mode (no vision) with the real robot:
	roslaunch sd02_joseph‑hoane_1 chess_robot.launch chess_robot:=test sim:=false robot_ip:=192.168.1.100

	In all cases:

	– sim:=true|false toggles simulated vs. real‐hardware movement.
	– chess_robot:=test tells the node to enter test mode (human enters moves on CLI).

## Setup

### Install `jh1` as a python library

In your virtual environment, run:

```
pip install -e ./src

```
Google slides for progress: https://docs.google.com/presentation/d/1KjLXtcGOjq-ztTGnWDBwuNPuuBlknc9xajZgTyLYn-0/edit?usp=sharing
