#!/usr/bin/env python3

"""
Description: This class defines methods to receive the joint states and 
             command the UR10e with gripper
             Need  to simulate the robot using 
                rosrun isu_rel simulator.py
             or connect to the real robot by
                i) roslaunch ur_robot_driver ur10e_griper_bringup.launch
                ii) roslaunch robotiq_2f_gripper_control robotiq_action_server.launch

Author: Ling Tang ling@iastate.edu
Date: 2025-02-16
Version: 1.0
"""
import sys
import rospy
import actionlib
from sensor_msgs.msg import JointState

from robotiq_2f_gripper_msgs.msg import RobotiqGripperStatus, CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)


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

JOINT_NAMES_UR10E = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_NAMES_GRIPPER = [
    "finger_joint",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

class RobotUR10eGripper:
    def __init__(self, is_gripper_up = True):
        rospy.init_node("ur10e_gripper")
        
        self._timeout = rospy.Duration(5)
        
        # controller setup
        self._switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self._load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self._list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        
        try:
            self._switch_srv.wait_for_service(self._timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        # uses 2 OR 4 for sim with gazebo
        self._switch_controller(JOINT_TRAJECTORY_CONTROLLERS[0]) # only this controller is implemented 


        # gripper setup
        self._gripper_status = is_gripper_up
        rospy.loginfo(f"_gripper_status: {self._gripper_status}")
            # Initialize the joint states by waiting for the first message
        if(self._gripper_status):
            try:
                # Add timeout of 2 seconds to wait_for_message
                data_gripper = rospy.wait_for_message("/gripper_joint_states", JointState, timeout=2.0)
                self._pos_gripper = data_gripper.position
                self._vel_gripper = data_gripper.velocity
            except rospy.ROSException as e:
                rospy.logwarn(f"Timeout waiting for gripper joint states: {e}")
                # Initialize with default values
                self._pos_gripper = [0.0]  # Default to open position
                self._vel_gripper = [0.0]
                
            rospy.Subscriber("/gripper_joint_states", JointState, self._callback_gripper_joint_state)
            self._robotiq_client = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
        
        rospy.loginfo("gripper setup finished.")
        
        # ur10e setup
        try:
            data_ur10e = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
            self._pos_ur10e = data_ur10e.position
            self._vel_ur10e = data_ur10e.velocity
        except rospy.ROSException as e:
            rospy.logwarn(f"Timeout waiting for ur10e joint states: {e}")
            # Initialize with default values
            self._pos_ur10e = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default positions
            self._vel_ur10e = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
        rospy.Subscriber("/joint_states" , JointState, self._callback_ur10e_joint_state)

        self._ur10e_client = actionlib.SimpleActionClient(
            JOINT_TRAJECTORY_CONTROLLERS[0] + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        rospy.loginfo("ur10e setup finished.")

    def command_robot(self, joint_angles, duration) -> bool:
        # check if joint_angles is valid
        if(self._gripper_status):
            if(len(joint_angles) != 7):
                rospy.logerr("Joint angles should have 7 elements: 6 for UR10e and 1 for gripper")
                return False
        else:
            if(len(joint_angles) != 6):
                rospy.logerr("Joint angles should have 6 elements: 6 for UR10e")
                return False
            
        self._command_ur10e(joint_angles[:6], duration)
        if(self._gripper_status):
            self._command_gripper(joint_angles[6])
            rospy.loginfo(f"Finished command gripper")
            return True
        
        rospy.logwarn(f"[command_robot] command_ur10e was successful, but gripper status was false")
        return False


    def _command_ur10e(self, joint_angles, duration) -> bool:
        rospy.loginfo(f"Attempting to command ur10e at {joint_angles=} {duration=}")
        if(len(joint_angles) != 6):
            rospy.logerr("Joint angles should have 6 for UR10e")
            return False

        timeout = rospy.Duration(5)
        if not self._ur10e_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES_UR10E

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.velocities = [0.0] * len(joint_angles)
        point.time_from_start = rospy.Duration(duration)

        goal.trajectory.points.append(point)
        self._ur10e_client.send_goal(goal)
        self._ur10e_client.wait_for_result()

        result = self._ur10e_client.get_result()
        success = (result.error_code == 0)  # 0 typically means SUCCESS
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))    
        return success

    def _command_gripper(self, position, speed = 0.05, force = 0) -> bool:
        rospy.loginfo(f"Attempting to command gripper at {position=}")
        goal = CommandRobotiqGripperGoal()
        goal.position = position
        goal.emergency_release = False
        goal.speed = speed
        goal.force = force
        self._robotiq_client.send_goal(goal)
        self._robotiq_client.wait_for_result()
        result = self._robotiq_client.get_result()
        if result is not None:
            status = self._robotiq_client.get_state()  # Get action state
            rospy.loginfo(f"Gripper command finished with status: {status}")
            return status == actionlib.GoalStatus.SUCCEEDED
        
        rospy.logerr("Gripper command failed - null result returned")
        return False

    def get_joint_pos(self):
        
        if(self._gripper_status):
            return self._pos_ur10e + self._pos_gripper
        else:
            return self._pos_ur10e
    
    def get_joint_vel(self):
        if(self._gripper_status):
            return self._vel_ur10e + self._vel_gripper
        else:
            return self._vel_ur10e
        
    def _callback_gripper_status(self, data):
        self._is_ready = data.is_ready
        self._is_reset = data.is_reset
        self._is_moving = data.is_moving
        self.obj_detected = data.obj_detected
        self._requested_position = data.requested_position

    def _callback_gripper_joint_state(self, data):
        self._pos_gripper = data.position
        self._vel_gripper = data.velocity
    
    def _callback_ur10e_joint_state(self, data):
        self._pos_ur10e = data.position
        self._vel_ur10e = data.velocity

    def get_info(self):
        robot_state = {}
        robot_state["joint_names"] = (JOINT_NAMES_UR10E + JOINT_NAMES_GRIPPER)
        robot_state["joint_positions"] = self.get_joint_pos()
        robot_state["joint_velocities"] = self.get_joint_vel()
        # robot_state["gripper:is_ready"] = self._is_ready
        # robot_state["gripper:is_reset"] = self._is_reset
        # robot_state["gripper:is_moving"] = self._is_moving
        # robot_state["gripper:obj_detected"] = self.obj_detected
        return robot_state
    
    def _switch_controller(self, target_controller):

        if(target_controller != JOINT_TRAJECTORY_CONTROLLERS[0]):
            rospy.logerr("Only scaled_pos_joint_traj_controller is implemented!")
            return

        self._ctrl_active = target_controller
        other_ctrl = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )
        other_ctrl.remove(target_controller)
        rospy.loginfo("List controllers request...")
        srv = ListControllersRequest()
        resp = self._list_srv(srv)
        rospy.loginfo(f"response from request: {resp}")
        
        for controller in resp.controller:
            if controller.name == target_controller and controller.state == "running":
                rospy.loginfo("Returning from _switch_controller()")
                return
            
        srv = LoadControllerRequest()
        print("Load controller: ", target_controller)
        srv.name = target_controller
        self._load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_ctrl
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self._switch_srv(srv)