# import rospy
# from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as GripperCmd
# import time

# class GripperController:
#     def __init__(self):
#         self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', GripperCmd, queue_size=1)
#         self.command = GripperCmd()
#         self.activated = False
#         self.last_activation_time = None
        
#     def activate(self):
#         """Activate the gripper"""
#         self.command.rACT = 1
#         self.command.rGTO = 1
#         self.command.rSP  = 255  # Speed
#         self.command.rFR  = 150  # Force
#         self.pub.publish(self.command)
#         self.activated = True
#         self.last_activation_time = time.time()
#         rospy.sleep(1)  # Give time for activation
        
#     def check_activation(self):
#         """Check if gripper needs reactivation (runs every 1-5 min)"""
#         if self.last_activation_time and time.time() - self.last_activation_time > 60:
#             rospy.loginfo("Preventative gripper reactivation")
#             self.activate()
            
#     def close(self, position=255):
#         """Close gripper to given position (255 = fully closed)"""
#         if not self.activated:
#             self.activate()
#         self.check_activation()
#         self.command.rPR = position
#         self.pub.publish(self.command)
#         rospy.sleep(0.5)
        
#     def open(self):
#         """Open gripper fully"""
#         if not self.activated:
#             self.activate()
#         self.check_activation()
#         self.command.rPR = 0
#         self.pub.publish(self.command)
#         rospy.sleep(0.5)

import rospy
import time

# Create simulation mode
SIMULATION_MODE = True
rospy.logwarn("Using simulated gripper interface")

class GripperCmd:
    def __init__(self):
        self.rACT = 0
        self.rGTO = 0
        self.rATR = 0
        self.rPR = 0  # Position request (0-255)
        self.rSP = 0  # Speed (0-255)
        self.rFR = 0  # Force (0-255)

class GripperController:
    def __init__(self):
        rospy.loginfo("Gripper running in SIMULATION mode")
        self.command = GripperCmd()
        self.activated = False
        self.last_activation_time = None
        
    def activate(self):
        """Activate the gripper"""
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP  = 255  # Speed
        self.command.rFR  = 150  # Force
        rospy.loginfo("SIM: Activating gripper")
        self.activated = True
        self.last_activation_time = time.time()
        rospy.sleep(1)  # Give time for activation
        
    def check_activation(self):
        """Check if gripper needs reactivation (runs every 1-5 min)"""
        if self.last_activation_time and time.time() - self.last_activation_time > 60:
            rospy.loginfo("SIM: Preventative gripper reactivation")
            self.activate()
            
    def close(self, position=255):
        """Close gripper to given position (255 = fully closed)"""
        if not self.activated:
            self.activate()
        self.check_activation()
        self.command.rPR = position
        rospy.loginfo(f"SIM: Closing gripper to position {position}")
        rospy.sleep(0.5)
        
    def open(self):
        """Open gripper fully"""
        if not self.activated:
            self.activate()
        self.check_activation()
        self.command.rPR = 0
        rospy.loginfo("SIM: Opening gripper")
        rospy.sleep(0.5)