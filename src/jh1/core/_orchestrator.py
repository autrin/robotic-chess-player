from typing import Optional

import numpy as np
import multiprocessing as mp
import time

from jh1.robotics.kinematics import JointVector
from jh1.topology import *
from jh1.robotics import Skeleton
from jh1.utils.visualize import animate_joint_vectors


class Orchestrator:
    def __init__(
        self,
        skeleton: Skeleton,
        require_viz: bool = True,
        require_approval: bool = True,
        min_duration: float = 2.0,
        max_duration: float = 8.0,
        std_duration: float = 3.0,
        angular_rads_per_second: float = 0.3
    ):
        self.skeleton: Skeleton = skeleton
        self.require_viz = require_viz
        self.require_approval = require_approval
        self.min_duration = min_duration
        self.max_duration = max_duration
        self.std_duration = std_duration
        self.angular_rads_per_second = angular_rads_per_second

    def set_minimum_duration(self, min_duration_secs: float) -> 'Orchestrator':
        self.min_duration = min_duration_secs
        return self

    def free_movement_sequence(self, start_square: str, end_square: str) -> bool:
        # Simply move the piece to the desired end square
        return self.pick_and_drop_action_chain(
            start_down=WAYPOINT_TABLE[start_square],
            end_down=WAYPOINT_TABLE[end_square],
            start_up=WAYPOINT_TABLE[start_square + UP_LABEL_SUFFIX],
            end_up=WAYPOINT_TABLE[end_square + UP_LABEL_SUFFIX],
            start_home=HOME_WAYPOINT,
            end_home=HOME_WAYPOINT
        )

    def capture_sequence(self, start_square: str, end_square: str) -> bool:
        start_w_down = WAYPOINT_TABLE[start_square]
        end_w_down = WAYPOINT_TABLE[end_square]
        start_w_up = WAYPOINT_TABLE[start_square + UP_LABEL_SUFFIX]
        end_w_up = WAYPOINT_TABLE[end_square + UP_LABEL_SUFFIX]

        # Pick up the captured piece and discard it
        if not self.pick_and_drop_action_chain(
            start_down=end_w_down,
            end_down=DISCARD_WAYPOINT,
            start_up=end_w_up,
            end_up=DISCARD_UP_WAYPOINT,
            start_home=HOME_WAYPOINT,
            end_home=None
        ): return False

        # Move the capturing piece from start to end square without returning to home
        return self.pick_and_drop_action_chain(
            start_down=start_w_down,
            end_down=end_w_down,
            start_up=start_w_up,
            end_up=end_w_up,
            start_home=None,
            end_home=HOME_WAYPOINT
        )

    def castling_sequence(self, is_white: bool, is_long_castles: bool) -> bool:
        rank = 1 if is_white else 8
        if is_long_castles:
            king_start_sq, king_end_sq = f"e{rank}", f"c{rank}"
            rook_start_sq, rook_end_sq = f"a{rank}", f"d{rank}"
        else:
            king_start_sq, king_end_sq = f"e{rank}", f"g{rank}"
            rook_start_sq, rook_end_sq = f"h{rank}", f"f{rank}"

        if not self.pick_and_drop_action_chain(
            start_down=WAYPOINT_TABLE[king_start_sq],
            end_down=WAYPOINT_TABLE[king_end_sq],
            start_up=WAYPOINT_TABLE[king_start_sq + UP_LABEL_SUFFIX],
            end_up=WAYPOINT_TABLE[king_end_sq + UP_LABEL_SUFFIX],
            start_home=HOME_WAYPOINT,
            end_home=None
        ): return False

        return self.pick_and_drop_action_chain(
            start_down=WAYPOINT_TABLE[rook_start_sq],
            end_down=WAYPOINT_TABLE[rook_end_sq],
            start_up=WAYPOINT_TABLE[rook_start_sq + UP_LABEL_SUFFIX],
            end_up=WAYPOINT_TABLE[rook_end_sq + UP_LABEL_SUFFIX],
            start_home=None,
            end_home=HOME_WAYPOINT
        )

    def en_passant_sequence(
        self,
        start_square: str,
        end_square: str
    ) -> bool:
        # In en passant, the captured pawn previously moved from rank 2 to 4 or 7 to 5, the pawn
        # capturing it will move from rank 4 to 3 or 5 to 6. Hence, the captured pawn's square will
        # be the file of the end square cross the rank of the start square.
        captured_square = end_square[0] + start_square[1]

        if not self.pick_and_drop_action_chain(
            start_down=WAYPOINT_TABLE[captured_square],
            end_down=DISCARD_WAYPOINT,
            start_up=WAYPOINT_TABLE[captured_square + UP_LABEL_SUFFIX],
            end_up=DISCARD_UP_WAYPOINT,
            start_home=HOME_WAYPOINT,
            end_home=None
        ): return False

        return self.pick_and_drop_action_chain(
            start_down=WAYPOINT_TABLE[start_square],
            end_down=WAYPOINT_TABLE[end_square],
            start_up=WAYPOINT_TABLE[start_square + UP_LABEL_SUFFIX],
            end_up=WAYPOINT_TABLE[end_square + UP_LABEL_SUFFIX],
            start_home=None,
            end_home=HOME_WAYPOINT
        )

    def pick_and_drop_action_chain(
        self,
        start_down: Waypoint,
        end_down: Waypoint,
        start_up: Waypoint,
        end_up: Waypoint,
        start_home: Optional[Waypoint] = HOME_WAYPOINT,
        end_home: Optional[Waypoint] = HOME_WAYPOINT,
    ) -> bool:
        if self.require_viz:
            print("\n\nDisplaying proposed robot movement sequence")
            ik_sequence = [
                start_home.jv,
                start_up.jv,
                start_down.jv,
                start_up.jv,
                end_up.jv,
                end_down.jv,
                end_up.jv,
                start_home.jv,
            ]

            # Display proposed movement
            time.sleep(1.0)
            p = mp.Process(
                target=animate_joint_vectors,
                args=(ik_sequence,)
            )
            p.start()
            time.sleep(1.0)

        if self.require_approval:
            print("\n\nPlease approve the proposed movement (y/n):")
            if not Orchestrator.prompt_approval():
                print("\nProposed movement has been rejected, please manually carry out the move.")
                return False
            print("\nMovement approved, proceeding...")

        # Move to home position to start
        if start_home is not None:
            if not self.skeleton.issue_arm_command(
                joint_vector=start_home.jv,
                duration=self.std_duration
            ): return False

        # Move to starting square, up position, open gripper
        if not self.skeleton.issue_aggregated_command(
            joint_vector=start_up.jv,
            gripper_span=Skeleton.GRIPPER_OPEN_POSITION,
            duration=self.compute_duration(start_up, self.angular_rads_per_second)
        ): return False

        # Grab the piece
        if not self.descend_grip_ascend_chain(
            up=start_up,
            down=start_down,
            new_gripper_span=Skeleton.GRIPPER_CLOSED_POSITION
        ): return False

        # Move to destination, up position
        if not self.skeleton.issue_arm_command(
            joint_vector=end_up.jv,
            duration=self.compute_duration(end_up, self.angular_rads_per_second)
        ): return False

        # Drop the piece
        if not self.descend_grip_ascend_chain(
            up=end_up,
            down=end_down,
            new_gripper_span=Skeleton.GRIPPER_OPEN_POSITION
        ): return False

        # Move back to home position to end
        if end_home is not None:
            return self.skeleton.issue_arm_command(
                joint_vector=end_home.jv,
                duration=self.std_duration
            )
        return True

    def descend_grip_ascend_chain(
        self,
        up: Waypoint,
        down: Waypoint,
        new_gripper_span: float
    ) -> bool:
        # Descend to down position
        if not self.skeleton.issue_arm_command(
            joint_vector=down.jv,
            duration=self.std_duration
        ): return False

        # Issue new gripper span
        if not self.skeleton.issue_gripper_command(
            gripper_span=new_gripper_span,
            speed=0.05
        ): return False

        # Ascend back to up position
        return self.skeleton.issue_arm_command(
            joint_vector=up.jv,
            duration=self.std_duration
        )

    def compute_duration(self, end: Waypoint, rads_per_second: float) -> float:
        if self.skeleton.configuration_vector is None: return self.std_duration
        metric_d = np.linalg.norm(self.skeleton.configuration_vector.as_np() - end.jv.as_np())
        print(f"[Orchestrator.compute_duration] Movement has a C-space distance of {metric_d=}")
        return np.clip(
            a=metric_d / rads_per_second,
            a_min=self.min_duration,
            a_max=self.max_duration
        )

    @staticmethod
    def prompt_approval() -> bool:
        while True:
            res = input(">> ")
            if res.lower() == "y":
                return True
            else:
                return False
