from dataclasses import dataclass

from typing import Tuple


@dataclass(frozen=True)
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

    @staticmethod
    def from_list(joint_list):
        if len(joint_list) != 6:
            raise ValueError("UR10e has exactly 6 joints.")
        return JointVector(*joint_list)
