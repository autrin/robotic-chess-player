#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import time
# no need to use this file. chessMovementController uses ur10eripper file that was sent by client
# the simulation part is usefull to be used in chessMovementController later

class RobotArmController:
    def __init__(self):
        # Check if we're in simulation mode
        self.sim_mode = rospy.get_param('~sim', True)
        rospy.loginfo(f"Robot arm initializing in {'simulation' if self.sim_mode else 'real'} mode")
        
        # Initialize connection if not in simulation mode
        if not self.sim_mode:
            # Connect to the UR10e action server
            self.traj_client = actionlib.SimpleActionClient(
                'scaled_pos_joint_traj_controller/follow_joint_trajectory', 
                FollowJointTrajectoryAction)
            
            rospy.loginfo("Waiting for UR10e action server...")
            server_exists = self.traj_client.wait_for_server(rospy.Duration(5.0))
            if server_exists:
                rospy.loginfo("Connected to UR10e action server")
            else:
                rospy.logwarn("Could not connect to UR10e action server, will operate in simulation mode")
                self.sim_mode = True
        else:
            rospy.loginfo("Running in simulation mode - robot movements will be simulated")
            self.traj_client = None
            
        # Define joint names
        self.joint_names = [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint', 
            'wrist_2_joint', 
            'wrist_3_joint'
        ]
        
        # Pre-defined positions (in joint space)
        self.positions = {
            "home": [0, -1.57, 0, -1.57, 0, 0],
            "high": [0, -1.0, 0.5, -1.0, -1.57, 0],  # Safety position above board
            "low": [0, -1.2, 1.0, -1.3, -1.57, 0],   # Position for grabbing/placing pieces
            "backup": [0.5, -1.0, 1.0, -1.5, -1.57, 0]  # Backup position away from the board
        }
        
        # Current joint states
        self.current_joints = self.positions["home"]  # Default to home position in simulation
        
        # Subscribe to joint states if not in simulation
        if not self.sim_mode:
            self.joint_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_cb)
            # Wait for first joint state message
            rospy.loginfo("Waiting for joint state...")
            wait_start = rospy.Time.now()
            while self.current_joints is None and not rospy.is_shutdown():
                if (rospy.Time.now() - wait_start).to_sec() > 5.0:
                    rospy.logwarn("Joint state timeout, will use default positions")
                    self.current_joints = self.positions["home"]
                    break
                rospy.sleep(0.1)
            rospy.loginfo("Received joint state")

    def _joint_state_cb(self, msg):
        # Filter for only the arm joints (exclude gripper)
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        
        if len(positions) == 6:
            self.current_joints = positions

    def move_to_position(self, position_name, duration=2.0):
        """Move to a predefined position (home, high, low, backup)"""
        if position_name in self.positions:
            return self.move_to_joints(self.positions[position_name], duration)
        else:
            rospy.logerr(f"Position '{position_name}' not defined")
            return False

    def move_to_joints(self, joint_positions, duration=2.0):
        """Move to specific joint positions"""
        if len(joint_positions) != 6:
            rospy.logerr("Expected 6 joint positions")
            return False
        
        if self.sim_mode:
            # In simulation mode, just simulate movement
            rospy.loginfo(f"SIM: Moving to joint positions {joint_positions}")
            time.sleep(duration)  # Simulate movement time
            self.current_joints = joint_positions
            return True
            
        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(duration)
        traj.points.append(point)
        
        # Create goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        
        # Send goal and wait
        self.traj_client.send_goal(goal)
        success = self.traj_client.wait_for_result(rospy.Duration(duration + 5.0))
        
        if success:
            rospy.loginfo("Successfully moved to position")
            return True
        else:
            rospy.logerr("Failed to move to position")
            return False

    def move_to_xyz(self, x, y, z, pitch=3.14, duration=2.0):
        """
        Move end effector to specific XYZ position with downward orientation
        Note: This is a simplified version that requires inverse kinematics
        """
        if self.sim_mode:
            # In simulation mode, just simulate movement
            rospy.loginfo(f"SIM: Moving to position ({x}, {y}, {z})")
            time.sleep(duration)  # Simulate movement time
            return True
            
        # TODO: Implement inverse kinematics or use MoveIt for this
        # For now, just log that this would move to the position
        rospy.loginfo(f"Would move to position ({x}, {y}, {z})")
        return True

    def emergency_stop(self):
        """Emergency stop - cancel current goal"""
        if not self.sim_mode:
            self.traj_client.cancel_all_goals()
        rospy.logwarn("EMERGENCY STOP ACTIVATED")