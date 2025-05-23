#!/usr/bin/env python3

"""
Description: This class defines methods to receive the joint states and 
             command the UR10e with gripper
             Need  to simulate the robot using 
                rosrun isu_rel simulator.py
             or connect to the real robot by
                i) roslaunch ur_robot_driver ur10e_griper_bringup.launch
                ii) roslaunch robotiq_2f_gripper_control robotiq_action_server.launch

Author: Ling Tang & updated by Autrin Hakimi
Date: 2025-02-16
Version: 1.0
"""
import sys
import rospy
import actionlib
from sensor_msgs.msg import JointState
import subprocess
import roslaunch
import rosnode
import time
from robotiq_2f_gripper_msgs.msg import RobotiqGripperStatus, CommandRobotiqGripperFeedback, \
    CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal

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
    def __init__(self, is_gripper_up=True):
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

        self._switch_controller(
            JOINT_TRAJECTORY_CONTROLLERS[0])  # only this controller is implemented

        # gripper setup
        self._gripper_status = is_gripper_up
        # prepare a launcher UUID for restart
        self._uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self._uuid)
        self._gripper_launcher = None
        # always listen for gripper joint states
        rospy.Subscriber("/gripper_joint_states", JointState,
                            self._callback_gripper_joint_state)
        # Initialize the joint states by waiting for the first message
        if self._gripper_status:
            data_gripper = rospy.wait_for_message("/gripper_joint_states", JointState)
            self._pos_gripper = data_gripper.position
            self._vel_gripper = data_gripper.velocity
            # rospy.Subscriber("/robotiq_2f_gripper_msgs/RobotiqGripperStatus" , RobotiqGripperStatus, self._callback_gripper_status)
            self._robotiq_client = actionlib.SimpleActionClient('command_robotiq_action',
                                                                CommandRobotiqGripperAction)

        # ur10e setup
        data_ur10e = rospy.wait_for_message("/joint_states", JointState)
        self._pos_ur10e = data_ur10e.position
        self._vel_ur10e = data_ur10e.velocity
        rospy.Subscriber("/joint_states", JointState, self._callback_ur10e_joint_state)

        self._ur10e_client = actionlib.SimpleActionClient(
            JOINT_TRAJECTORY_CONTROLLERS[0] + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

    def __del__(self):
        if self._gripper_launcher:
            self._gripper_launcher.shutdown()

    def command_robot(self, joint_angles, duration) -> bool:
        # if they passed 7 angles (gripper included) but it’s offline, try to restart
        if len(joint_angles) == 7 and not self._gripper_status:
            rospy.logwarn("Gripper down at command time. Trying to restart…")
            if not self.turnon_gripper_using_ros():
                rospy.logerr("Could not bring up gripper; aborting command_robot()")
                return False

        # now validate length
        expected_joint_length = 7 if self._gripper_status else 6
        if len(joint_angles) != expected_joint_length:
            rospy.logerr(f"Expected {expected_joint_length} joint angles, got {len(joint_angles)}")
            return False

        self._command_ur10e(joint_angles[:6], duration)
        if self._gripper_status:
            self._command_gripper(joint_angles[6])
            rospy.loginfo(f"Finished command gripper")
            return True
        else:
            # gripper_on = self.turnon_gripper()
            # if gripper_on: 
            #     self._command_gripper(joint_angles[6])
            #     rospy.loginfo(f"Finished command gripper")
            #     return True
            # else:
            rospy.logwarn(f"[command_robot] command_ur10e was successful, but gripper status was false")
            return False

    def _command_ur10e(self, joint_angles, duration) -> bool:
        # rospy.loginfo(f"Attempting to command ur10e at {joint_angles=} {duration=}")
        if len(joint_angles) != 6:
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

    def _command_gripper(self, position, speed=0.05, force=0) -> bool:
        # rospy.loginfo(f"Attempting to command gripper at {position=}")
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

        if self._gripper_status:
            return self._pos_ur10e + self._pos_gripper
        else:
            return self._pos_ur10e

    def get_joint_vel(self):
        if self._gripper_status:
            return self._vel_ur10e + self._vel_gripper
        else:
            return self._vel_ur10e

    def _callback_gripper_status(self, data):
        self._is_ready = data.is_ready
        self._is_reset = data.is_resetf
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

        if target_controller != JOINT_TRAJECTORY_CONTROLLERS[0]:
            rospy.logerr("Only scaled_pos_joint_traj_controller is implemented!")
            return

        self._ctrl_active = target_controller
        other_ctrl = (
                JOINT_TRAJECTORY_CONTROLLERS
                + CARTESIAN_TRAJECTORY_CONTROLLERS
                + CONFLICTING_CONTROLLERS
        )
        other_ctrl.remove(target_controller)

        srv = ListControllersRequest()
        resp = self._list_srv(srv)

        for controller in resp.controller:
            if controller.name == target_controller and controller.state == "running":
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

    def turnon_gripper(self):
        if not self._gripper_status:
            rospy.loginfo("Gripper is offline. Attempting to restart it...")
            try:
                # Start the launch file in background
                subprocess.Popen(
                    ["roslaunch", "robotiq_2f_gripper_control", "robotiq_action_server.launch"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                # Give it time to start up
                rospy.loginfo("Waiting for gripper action server to start...")
                time.sleep(7)
                
                # Reinitialize the gripper connections
                try:
                    # Wait for gripper joint states
                    data_gripper = rospy.wait_for_message("/gripper_joint_states", JointState)
                    self._pos_gripper = data_gripper.position
                    self._vel_gripper = data_gripper.velocity
                    rospy.Subscriber("/gripper_joint_states", JointState, self._callback_gripper_joint_state)
                    # Recreate the action client
                    self._robotiq_client = actionlib.SimpleActionClient('command_robotiq_action',
                                                                    CommandRobotiqGripperAction)
                    
                    # Wait for the action server
                    server_connected = self._robotiq_client.wait_for_server(timeout=rospy.Duration(10.0))
                    
                    if server_connected:
                        self._gripper_status = True
                        rospy.loginfo("Successfully reconnected to gripper!")
                        return True
                    else:
                        rospy.logerr("Timed out waiting for gripper action server")
                        return False
                    
                except rospy.ROSException as e:
                    rospy.logerr(f"Failed to reconnect to gripper: {str(e)}")
                    return False
                    
            except Exception as e:
                rospy.logerr(f"Failed to restart gripper action server: {str(e)}")
                return False
        
        return True  # Gripper is already online
    
    def turnon_gripper_using_ros(self):
        # 1) kill any stale node
        try:
            rosnode.kill_nodes(['/robotiq_action_server'])
        except rosnode.ROSNodeIOException:
            pass
        # 2) tear down old launcher
        if self._gripper_launcher:
            self._gripper_launcher.shutdown()
            self._gripper_launcher = None

        # 3) start fresh
        rospy.loginfo("Restarting gripper driver...")
        self._gripper_launcher = roslaunch.parent.ROSLaunchParent(
            self._uuid, ["robotiq_2f_gripper_control/robotiq_action_server.launch"])
        self._gripper_launcher.start()

        # 4) resubscribe to joint states
        # rospy.Subscriber("/gripper_joint_states", JointState,
        #                  self._callback_gripper_joint_state)
        
        # 5) wait for action server
        self._robotiq_client = actionlib.SimpleActionClient(
            'command_robotiq_action', CommandRobotiqGripperAction
        )
        if not self._robotiq_client.wait_for_server(rospy.Duration(15.0)):
            rospy.logerr("Gripper action server never appeared")
            return False
        
        # 6) sanity check by waiting for first join state
        try:
            data = rospy.wait_for_message("/gripper_joint_states", JointState, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("Gripper join_states never came back")
            return False
        
        # 7) finally mark online
        self._pos_gripper = data.position
        self._vel_gripper = data.velocity
        self._gripper_status = True
        rospy.loginfo("Gripper is back online!")
        return True