from dataclasses import dataclass

from jh1.robotics.kinematics import JointVector
from jh1.typealias import Vec3


@dataclass
class Waypoint:
    label: str
    pos: Vec3
    jv: JointVector