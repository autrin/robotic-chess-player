#!/usr/bin/env python3
import rospy
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_msgs.msg import RobotiqGripperStatus
import time

class GripperController:
    def __init__(self):
        # Check if we're in simulation mode
        self.sim_mode = rospy.get_param('~sim', True)
        rospy.loginfo(f"Gripper initializing in {'simulation' if self.sim_mode else 'real'} mode")
        
        # Connect to the Robotiq action server
        if not self.sim_mode:
            self.gripper_client = actionlib.SimpleActionClient(
                'command_robotiq_action', 
                CommandRobotiqGripperAction)
            
            rospy.loginfo("Waiting for gripper action server...")
            server_exists = self.gripper_client.wait_for_server(rospy.Duration(5.0))
            if server_exists:
                rospy.loginfo("Connected to gripper action server")
            else:
                rospy.logwarn("Could not connect to gripper action server, will operate in simulation mode")
                self.sim_mode = True
        else:
            rospy.loginfo("Running in simulation mode - gripper movements will be simulated")
            self.gripper_client = None
        
        # Gripper status
        self.gripper_status = None
        self.gripper_position = 0.0  # Fully open
        
        # Only subscribe to status in real mode
        if not self.sim_mode:
            self.status_sub = rospy.Subscriber(
                'robotiq_2f_gripper_status',
                RobotiqGripperStatus,
                self._status_cb)
        
        self.activated = self.sim_mode  # In sim mode, we're always "activated"
        self.last_activation_time = time.time() if self.sim_mode else None
        
    def _status_cb(self, msg):
        self.gripper_status = msg
        
    def activate(self):
        """Activate the gripper"""
        if self.sim_mode:
            rospy.loginfo("SIM: Gripper activated")
            self.activated = True
            self.last_activation_time = time.time()
            return True
            
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
        if self.sim_mode:
            return True
            
        if not self.activated or (self.last_activation_time and time.time() - self.last_activation_time > 60):
            rospy.loginfo("Preventative gripper reactivation")
            return self.activate()
        return True
            
    def close(self, position=0.5):
        """Close gripper (position 0.0-1.0)"""
        if self.sim_mode:
            rospy.loginfo(f"SIM: Gripper closing to position {position}")
            self.gripper_position = position
            time.sleep(0.5)  # Simulate movement time
            return True
            
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