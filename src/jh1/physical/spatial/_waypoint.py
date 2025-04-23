from jh1.typealias import *


class Waypoint:
    def __init__(self, uid: int, label: str, local_pos: Vec3):
        self.uid: int = uid
        self.label: str = label
        self.local_pos: Vec3 = local_pos

