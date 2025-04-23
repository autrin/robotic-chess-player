#!/usr/bin/env python3

"""
Description: This class defines methods to publish the joint states and 
             receive commands to simulate the UR10e with gripper

Author: Ling Tang
Date: 2025-02-16
Version: 1.0
"""

import rospy
import actionlib
from sensor_msgs.msg import JointState

from robotiq_2f_gripper_msgs.msg import RobotiqGripperStatus, CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerResponse, SwitchController
from controller_manager_msgs.srv import LoadControllerResponse, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersResponse
from controller_manager_msgs.msg import ControllerState

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)


import random

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]



class RobotSim:
    def __init__(self):
        rospy.init_node("robot_sim")
        self._joint_names =[ 
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint", 
            "finger_joint"]
        
        self._ctrl_active = "forward_joint_traj_controller"
        self._gripper_joint_state_pub = rospy.Publisher("/gripper_joint_states" , JointState, queue_size=10)
        self._gripper_status_pub = rospy.Publisher("/robotiq_2f_gripper_msgs/RobotiqGripperStatus" , RobotiqGripperStatus, queue_size=10)
        self._ur10e_joint_state_pub = rospy.Publisher("/joint_states" , JointState, queue_size=10)
        
        self._gripper_action_server = actionlib.SimpleActionServer('command_robotiq_action', 
                                                            CommandRobotiqGripperAction, 
                                                            execute_cb=self._gripper_execute_cb, 
                                                            auto_start = False)
        
        self._ur10e_action_server = actionlib.SimpleActionServer(self._ctrl_active  + "/follow_joint_trajectory", 
                                            FollowJointTrajectoryAction, 
                                            execute_cb=self._ur10e_execute_cb, 
                                            auto_start = False)
        
        # provide service to switch controller
        rospy.Service('controller_manager/switch_controller', SwitchController, self._switch_controller)
        rospy.Service('controller_manager/load_controller', LoadController, self._load_controller)
        rospy.Service('controller_manager/list_controllers', ListControllers, self._list_controllers)


        if(not rospy.is_shutdown()):
            self._gripper_action_server.start()
            self._ur10e_action_server.start()


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._gripper_joint_state_pub.publish(self._get_gripper_joint_state())
            self._gripper_status_pub.publish(self._get_gripper_status())

            self._ur10e_joint_state_pub.publish(self._get_ur10e_joint_state())
            rate.sleep()

    def _ur10e_execute_cb(self, goal):
        rospy.loginfo("Received ur10e command: " + str(goal.trajectory.points[0].positions)) 
        result = FollowJointTrajectoryResult()
        rospy.sleep(goal.trajectory.points[0].time_from_start.to_sec())
        self._ur10e_action_server.set_succeeded(result)

    def _gripper_execute_cb(self, goal):
        rospy.loginfo("Received gripper command: " + str(goal.position)) 
        result = self._get_gripper_status()
        rospy.sleep(0.5)
        self._gripper_action_server.set_succeeded(result)

    def _get_gripper_status(self):
        data = RobotiqGripperStatus()
        data.header.stamp = rospy.get_rostime()
        data.header.seq = 0
        data.is_ready = True
        data.is_reset = False
        data.is_moving = False
        data.obj_detected = False
        data.position = random.uniform(0.1, 0.9)
        data.requested_position = random.uniform(0.1, 0.9)
        data.current = random.uniform(0.1, 0.9)
        return data

    def _get_gripper_joint_state(self):
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = 0
        js.name = [self._joint_names[-1]]
        js.position = [random.uniform(0.1, 0.9)] # fake joint position
        js.velocity = [random.uniform(0.1, 0.9)] # fake joint velocity
        return js
    
    def _get_ur10e_joint_state(self):
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = 0
        js.name = self._joint_names[:-1]
        js.position = [random.uniform(0.1, 0.9) for _ in range(6)]
        js.velocity = [random.uniform(0.1, 0.9) for _ in range(6)]
        return js
    
    def _switch_controller(self, req):

        
        self._ctrl_active = req.start_controllers[0]
        rospy.loginfo("Switch to controller: %s", req.start_controllers[0])


        self._ur10e_action_server = actionlib.SimpleActionServer(self._ctrl_active  + "/follow_joint_trajectory", 
                                            FollowJointTrajectoryAction, 
                                            execute_cb=self._ur10e_execute_cb, 
                                            auto_start = False)
        
        self._ur10e_action_server.start()

        res = SwitchControllerResponse()
        res.ok = True
        return res
    

    
    def _load_controller(self, req):

        rospy.loginfo("Load controller: %s", req.name)

        res = LoadControllerResponse()
        res.ok = True
        return res
    
    def _list_controllers(self, req):
        rospy.loginfo("List controllers")

        res = ListControllersResponse()
        
        for i, name in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
            joint_ctrl = ControllerState()
            joint_ctrl.name = name
            joint_ctrl.state = "running" if name == self._ctrl_active else "stopped"
            res.controller.append(joint_ctrl)

        return res


if __name__ == '__main__':
    try:
        robot_simulator = RobotSim()
        robot_simulator.run()
    except rospy.ROSInterruptException:
          pass
    