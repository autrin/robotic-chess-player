from dataclasses import dataclass

import numpy as np

from jh1.typealias import *

from typing import Tuple, List


@dataclass
class JointVector:
    shoulder_pan: float
    shoulder_lift: float
    elbow: float
    wrist_1: float
    wrist_2: float
    wrist_3: float

    def adaptive_leveling(self) -> 'JointVector':
        return JointVector(
            self.shoulder_pan,
            self.shoulder_lift,
            self.elbow,
            3 * np.pi / 2 - ( self.shoulder_lift + self.elbow),
            # 0,
            -np.pi / 2,
            -np.pi / 2
        )

    def as_list(self) -> List[float]:
        return [
            self.shoulder_pan,
            self.shoulder_lift,
            self.elbow,
            self.wrist_1,
            self.wrist_2,
            self.wrist_3,
        ]

    def as_np(self) -> NDArray[float64]:
        return np.array(self.as_list())

    def as_command(self, gripper: float) -> List[float]:
        return [
            self.elbow,
            self.shoulder_lift,
            self.shoulder_pan,
            self.wrist_1,
            self.wrist_2,
            self.wrist_3,
            gripper
        ]

    @staticmethod
    def from_list(joint_list):
        if len(joint_list) != 6:
            raise ValueError("UR10e has exactly 6 joints.")
        return JointVector(*joint_list)

    @staticmethod
    def from_topic(joint_list):
        """
        The order that the subscriber outputs is different from the expected order. Specifically:
            - elbow_joint
            - shoulder_lift_joint
            - shoulder_pan_joint
            - wrist_1_joint
            - wrist_2_joint
            - wrist_3_joint
        """
        if len(joint_list) != 6:
            raise ValueError("UR10e has exactly 6 joints.")
        return JointVector(
            joint_list[2],
            joint_list[1],
            joint_list[0],
            joint_list[3],
            joint_list[4],
            joint_list[5]
        )

    JOINT_LABELS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
