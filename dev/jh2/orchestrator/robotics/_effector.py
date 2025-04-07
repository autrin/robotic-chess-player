import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from jh2.orchestrator.robotics import Armature
from jh2.orchestrator.robotics._constants import *


class Effector:
    def __init__(self, parent: Armature):
        self.parent = parent
        self.gripper_joint_name = HAND_E_GRIPPER_JOINT_NAME
        self.gripper_pub = rospy.Publisher(
            GRIPPER_COMMAND_TOPIC, JointTrajectory, queue_size=10
        )

    def open(self):
        """Opens the gripper."""
        rospy.loginfo("Opening gripper...")
        self._send_gripper_command(position=GRIPPER_OPEN_POSITION)

    def close(self):
        """Closes the gripper."""
        rospy.loginfo("Closing gripper...")
        self._send_gripper_command(position=GRIPPER_CLOSED_POSITION)

    def _send_gripper_command(self, position: float):
        """Publishes a trajectory to move the gripper to the given joint position."""
        traj = JointTrajectory()
        traj.joint_names = [self.gripper_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.effort = []
        point.time_from_start = rospy.Duration(GRIPPER_MOVE_DURATION)

        traj.points.append(point)

        self.gripper_pub.publish(traj)
        rospy.sleep(GRIPPER_MOVE_DURATION)
