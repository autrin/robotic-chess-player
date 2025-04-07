from jh2.orchestrator.robotics import Armature


class RobotOrchestrator:
    def __init__(self, armature: Armature, effector: Effector):
        self.armature = armature
        self.effector = effector
