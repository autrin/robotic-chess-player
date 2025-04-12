import rospy
import moveit_commander
from tf.transformations import quaternion_from_euler
import time

class RobotArmController:
    def __init__(self):
        self.group, self.robot, self.scene = self._init_robot()
        self.positions = {
            "high": [0, -0.5, 0.5, -0.5, -1.57, 0],  # Safety position above board
            "low": [0, -1.0, 1.0, -0.5, -1.57, 0],   # Position for grabbing/placing pieces
            "backup": [0.5, -1.0, 1.0, 0, -1.57, 0]  # Backup position away from the board
        }
        
    def _init_robot(self):
        # Initialize MoveIt and ROS
        moveit_commander.roscpp_initialize([])
        rospy.init_node("chess_robot", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("manipulator")
        group.set_max_velocity_scaling_factor(0.5)
        group.set_max_acceleration_scaling_factor(0.5)
        return group, robot, scene

    def move_to_position(self, position_name):
        """Move to a predefined position (high, low, backup)"""
        if position_name in self.positions:
            self.group.go(self.positions[position_name], wait=True)
            self.group.stop()
            return True
        return False

    def emergency_stop(self):
        """Emergency stop - halt all movement"""
        self.group.stop()
        rospy.logwarn("EMERGENCY STOP ACTIVATED")
        
        # we may need additional code here to release motors
        # or take other safety actions depending on the hardware    
        
    # More methods will be added for chess movements