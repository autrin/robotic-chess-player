from jh2.orchestrator.robotics import Armature


class Effector:
    def __init__(self, parent: Armature):
        self.parent = parent