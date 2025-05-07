# sd02_joseph-hoane_1
## Setup

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
	
	source ~/Desktop/402project/project/catkin_ws/devel/setup.bash

	1. Run robot driver
		roslaunch ur_robot_driver ur10e_griper_bringup.launch
	2. Access external control

	3. Run gripper driver (if not initialized don't do anyting)
		roslaunch robotiq_2f_gripper_control robotiq_action_server.launch
	
	4. Run main
		rosrun sd02_joseph-hoane_1 main test
		or 
		rosrun sd02_joseph-hoane_1 main


### Install `jh1` as a python library

In your virtual environment, run:

```
pip install -e ./src

```
Google slides for progress: https://docs.google.com/presentation/d/1KjLXtcGOjq-ztTGnWDBwuNPuuBlknc9xajZgTyLYn-0/edit?usp=sharing


