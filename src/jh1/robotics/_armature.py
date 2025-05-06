# from .robot_ur10e_gripper import RobotUR10eGripper
from jh1.robotics.kinematics import JointVector, ur10e_forward_kinematics, ur10e_inverse_kinematics

from jh1.typealias import *


class Armature:
    """
    A wrapper around the base RobotUR10eGripper class
    """

    def __init__(self, robot: 'RobotUR10eGripper'):
        self.robot: 'RobotUR10eGripper' = robot

    @staticmethod
    def forward_kinematics(q: JointVector) -> NDArray[Vec3]:
        return ur10e_forward_kinematics(q.as_np())

    @staticmethod
    def inverse_kinematics(target: Vec3) -> JointVector:
        return ur10e_inverse_kinematics(
            target,
            initial_q=JointVector.from_list([2.2015, -1.7744, 1.1871, -2.0474, -1.5897, 2.0208]),
        )
