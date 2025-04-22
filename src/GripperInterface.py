#!/usr/bin/env python3
import rospy
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_msgs.msg import RobotiqGripperStatus
import time

class GripperController:
    def __init__(self):
        # Connect to the Robotiq action server
        self.gripper_client = actionlib.SimpleActionClient(
            'command_robotiq_action', 
            CommandRobotiqGripperAction)
        
        rospy.loginfo("Waiting for gripper action server...")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Connected to gripper action server")
        
        # Subscribe to gripper status
        self.gripper_status = None
        self.status_sub = rospy.Subscriber(
            '/robotiq_2f_gripper_msgs/RobotiqGripperStatus',
            RobotiqGripperStatus,
            self._status_cb)
        
        self.activated = False
        self.last_activation_time = None
        
    def _status_cb(self, msg):
        self.gripper_status = msg
        
    def activate(self):
        """Activate the gripper"""
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = 0.0  # Open position
        goal.speed = 1.0     # Max speed
        goal.force = 1.0     # Max force
        
        self.gripper_client.send_goal(goal)
        success = self.gripper_client.wait_for_result(rospy.Duration(5.0))
        
        if success:
            rospy.loginfo("Gripper activated")
            self.activated = True
            self.last_activation_time = time.time()
            return True
        else:
            rospy.logerr("Failed to activate gripper")
            return False
            
    def check_activation(self):
        """Check if gripper needs reactivation"""
        if not self.activated or (self.last_activation_time and time.time() - self.last_activation_time > 60):
            rospy.loginfo("Preventative gripper reactivation")
            return self.activate()
        return True
            
    def close(self, position=0.5):
        """Close gripper (position 0.0-1.0)"""
        if not self.check_activation():
            return False
            
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = position  # 0.0 is open, 1.0 is closed
        goal.speed = 1.0
        goal.force = 1.0
        
        self.gripper_client.send_goal(goal)
        success = self.gripper_client.wait_for_result(rospy.Duration(5.0))
        
        if success:
            rospy.loginfo(f"Gripper closed to position {position}")
            return True
        else:
            rospy.logerr("Failed to close gripper")
            return False
        
    def open(self):
        """Open gripper fully"""
        return self.close(0.0)  # 0.0 is fully open