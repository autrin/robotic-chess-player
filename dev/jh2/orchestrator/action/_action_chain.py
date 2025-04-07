from typing import List

from jh2.orchestrator.action._action import Action


class ActionChain:
    def __init__(self):
        self.actions: List[Action] = []
        pass

    def append(self, action: Action):
        self.actions.append(action)
        return self

