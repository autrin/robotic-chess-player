from typing import List

from jh2.orchestrator.action._abstract_action import AbstractAction


class ActionChain:
    def __init__(self):
        self.actions: List[AbstractAction] = []
        pass


    def then(self, action: AbstractAction):
        self.actions.append(action)
        return self

