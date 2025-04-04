import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

from ._constants import *
from ._posture import Posture


class Armature:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(ROS_NODE_NAME, anonymous=True)

        self.arm: moveit_commander.move_group.MoveGroupCommander
        self.arm = moveit_commander.MoveGroupCommander(ARMATURE_CMDR_GRP_NAME)

        self.arm.set_planning_time(MAXIMUM_PLANNING_TIME_SECS)
        # set reference frame for end-effectors (the gripper)
        self.arm.set_pose_reference_frame("base_link")
        self.arm.allow_replanning(True)

    def move_to_posture(self, posture: Posture):
        self.arm.set_joint_value_target(posture.as_tuple())
        success = self.arm.go(wait=True)

        if not success: rospy.logerr("Failed to reach target posture.")
        self.arm.stop()
        self.arm.clear_pose_targets()

    def get_current_posture(self) -> Posture:
        joint_values = self.arm.get_current_joint_values()
        return Posture.from_list(joint_values)

    def get_current_pose(self) -> Pose:
        return self.arm.get_current_pose().pose
