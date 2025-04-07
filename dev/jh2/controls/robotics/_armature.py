import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

from ._constants import *
from ._joint_vector import JointVector


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

    def move_to_posture(self, posture: JointVector):
        self.arm.set_joint_value_target(posture.as_tuple())
        success = self.arm.go(wait=True)

        if not success: rospy.logerr("Failed to reach target joint posture.")
        self.arm.stop()
        self.arm.clear_pose_targets()


    def move_to_pose(self, pose: Pose):
        """Moves the robot end-effector to a specified pose."""
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)

        if not success:
            rospy.logerr("Failed to reach target pose.")
        else:
            rospy.loginfo("Reached target pose successfully.")

        self.arm.stop()
        self.arm.clear_pose_targets()

    def get_current_posture(self) -> JointVector:
        joint_values = self.arm.get_current_joint_values()
        return JointVector.from_list(joint_values)

    def get_current_pose(self) -> Pose:
        return self.arm.get_current_pose().pose

    def move_to_position(self, x: float, y: float, z: float):
        current_pose = self.get_current_pose()

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation = current_pose.orientation

        self.move_to_pose(target_pose)