from jh1.physical.topology import SQUARE_IK_LOOKUP, HOME_WAYPOINT, Waypoint
from jh1.robotics import Armature
from jh1.utils.visualize import animate_joint_vectors


class Orchestrator:
    def __init__(
        self,
        armature: Armature,
        require_viz: bool = True,
        require_approval: bool = True,
        min_duration_secs: float = 1.25
    ):
        self.armature: Armature = armature
        self.require_viz = require_viz
        self.require_approval = require_approval
        self.min_duration_secs: float = min_duration_secs

    def set_minimum_duration(self, min_duration_secs: float) -> 'Orchestrator':
        self.min_duration_secs = min_duration_secs
        return self

    def pick_and_drop_sequence(
        self,
        start_square: str, end_square: str,
        home_waypoint: Waypoint = HOME_WAYPOINT
    ) -> bool:
        start_w = SQUARE_IK_LOOKUP[start_square]
        start_w_up = SQUARE_IK_LOOKUP[start_square + "_up"]
        end_w = SQUARE_IK_LOOKUP[end_square]
        end_w_up = SQUARE_IK_LOOKUP[end_square + "_up"]

        if self.require_viz:
            print("\n\nDisplaying proposed robot movement sequence")
            ik_sequence = [
                home_waypoint.angles_ik,
                start_w_up.angles_ik,
                start_w.angles_ik,
                start_w_up.angles_ik,
                end_w_up.angles_ik,
                end_w.angles_ik,
                end_w_up.angles_ik,
                home_waypoint.angles_ik,
            ]

            # Display proposed movement
            animate_joint_vectors(ik_sequence)

        if self.require_approval:
            print("\n\nPlease approve the proposed movement (y/n):")
            if not Orchestrator.prompt_approval(): return False

        return True

    @staticmethod
    def prompt_approval() -> bool:
        while True:
            res = input(">> ")
            if res == "y":
                return True
            elif res == "n":
                return False
            else:
                print("Unexpected response, try again...")
