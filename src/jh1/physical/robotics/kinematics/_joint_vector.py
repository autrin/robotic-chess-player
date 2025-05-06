from dataclasses import dataclass

import numpy as np

from jh1.typealias import *

from typing import Tuple


@dataclass
class JointVector:
    shoulder_pan: float
    shoulder_lift: float
    elbow: float
    wrist_1: float
    wrist_2: float
    wrist_3: float

    def as_tuple(self) -> Tuple[float, float, float, float, float, float]:
        return (
            self.shoulder_pan,
            self.shoulder_lift,
            self.elbow,
            self.wrist_1,
            self.wrist_2,
            self.wrist_3,
        )

    def as_np(self) -> NDArray[float64]:
        return np.array(self.as_tuple())

    @staticmethod
    def from_list(joint_list):
        if len(joint_list) != 6:
            raise ValueError("UR10e has exactly 6 joints.")
        return JointVector(*joint_list)
