# from .robot_ur10e_gripper import RobotUR10eGripper
from jh1.robotics.kinematics import JointVector, ur10e_forward_kinematics

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
        raise NotImplementedError()
