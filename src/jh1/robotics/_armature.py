from typing import Optional, Union

import numpy as np

# from .robot_ur10e_gripper import RobotUR10eGripper
from jh1.robotics.kinematics import JointVector, ur10e_forward_kinematics, ur10e_inverse_kinematics, \
    ur10e_adaptive_inverse_kinematics

from jh1.typealias import *


class Armature:
    """
    A wrapper around the base RobotUR10eGripper class
    """

    def __init__(self, robot: 'RobotUR10eGripper'):
        self.robot: 'RobotUR10eGripper' = robot

    def move_to(self, joint_vector: JointVector, gripper_span: float, duration: float) -> bool:
        return self.robot.command_robot(joint_vector.as_command(gripper_span), duration)

    @staticmethod
    def forward_kinematics(q: JointVector) -> NDArray[Vec3]:
        return ur10e_forward_kinematics(q.as_np())

    @staticmethod
    def inverse_kinematics(target: Vec3) -> JointVector:
        _2pi = 2 * np.pi
        _pi_over_2 = np.pi / 2
        eps = 1e-2

        return ur10e_inverse_kinematics(
            target,
            initial_q=JointVector.from_topic(
                [1.9908550421344202, -1.0797357720187684, 1.2676620483398438, -2.4606195888915003,
                 -_pi_over_2, _pi_over_2]),
            joint_lower_bounds=[-_2pi, -_2pi, -_2pi, -_2pi, -_pi_over_2 - eps, _pi_over_2 - eps],
            joint_upper_bounds=[_2pi, _2pi, _2pi, _2pi, -_pi_over_2 + eps, _pi_over_2 + eps]
        )

    @staticmethod
    def adaptive_inverse_kinematics(
            target: Vec3,
            initial_q_hat: Optional[Union[JointVector, np.ndarray, list]] = None
    ) -> JointVector:
        _2pi = 2 * np.pi
        _pi_over_2 = np.pi / 2
        eps = 1e-2

        return ur10e_adaptive_inverse_kinematics(
            target,
            initial_q_hat,
            joint_lower_bounds=[-_2pi, -_2pi, -_2pi, -_2pi, -_2pi, _pi_over_2 - eps],
            joint_upper_bounds=[_2pi, _2pi, _2pi, _2pi, -_2pi + eps, _pi_over_2 + eps]
        )
