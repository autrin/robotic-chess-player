from typing import Optional

import numpy as np
import rospy

from jh1.physical.topology import SQUARE_IK_LOOKUP, HOME_WAYPOINT, Waypoint
from jh1.robotics import Skeleton
from jh1.utils.visualize import animate_joint_vectors


class Orchestrator:
    def __init__(
        self,
        skeleton: Skeleton,
        require_viz: bool = True,
        require_approval: bool = True,
        min_duration: float = 1.25,
        max_duration: float = 8.0,
        std_duration: float = 3.0,
        speed_meters_per_sec: float = 0.065
    ):
        self.skeleton: Skeleton = skeleton
        self.require_viz = require_viz
        self.require_approval = require_approval
        self.min_duration = min_duration
        self.max_duration = max_duration
        self.std_duration = std_duration
        self.speed_meters_per_sec = speed_meters_per_sec

    def set_minimum_duration(self, min_duration_secs: float) -> 'Orchestrator':
        self.min_duration = min_duration_secs
        return self

    def free_movement_sequence(self, start_square: str, end_square: str):
        self.pick_and_drop_action_chain(
            start_w_down=SQUARE_IK_LOOKUP[start_square],
            end_w_down=SQUARE_IK_LOOKUP[end_square],
            start_w_up=SQUARE_IK_LOOKUP[start_square + "_up"],
            end_w_up=SQUARE_IK_LOOKUP[end_square + "_up"],
            home_waypoint=HOME_WAYPOINT
        )

    def pick_and_drop_action_chain(
        self,
        start_w_down: Waypoint,
        end_w_down: Waypoint,
        start_w_up: Waypoint,
        end_w_up: Waypoint,
        home_waypoint: Optional[Waypoint] = HOME_WAYPOINT,
    ) -> bool:
        if self.require_viz:
            print("\n\nDisplaying proposed robot movement sequence")
            ik_sequence = [
                home_waypoint.jv,
                start_w_up.jv,
                start_w_down.jv,
                start_w_up.jv,
                end_w_up.jv,
                end_w_down.jv,
                end_w_up.jv,
                home_waypoint.jv,
            ]

            # Display proposed movement
            animate_joint_vectors(ik_sequence)

        if self.require_approval:
            print("\n\nPlease approve the proposed movement (y/n):")
            if not Orchestrator.prompt_approval(): return False
            print("\nMovement approved, proceeding...")

        # Move to home position to start
        if home_waypoint is not None:
            rospy.loginfo("Moving arm to home position")
            self.skeleton.issue_arm_command(
                joint_vector=home_waypoint.jv,
                duration=self.std_duration
            )

        # Move to starting square, up position, open gripper
        self.skeleton.issue_aggregated_command(
            joint_vector=start_w_up.jv,
            gripper_span=Skeleton.GRIPPER_OPEN_POSITION,
            duration=self.compute_duration(home_waypoint, start_w_up, self.speed_meters_per_sec)
        )

        # Grab the piece
        self.descend_grip_ascend_chain(
            w_up=start_w_up,
            w_down=start_w_down,
            new_gripper_span=Skeleton.GRIPPER_CLOSED_POSITION
        )

        # Move to destination, up position
        self.skeleton.issue_arm_command(
            joint_vector=end_w_up.jv,
            duration=self.compute_duration(start_w_up, end_w_up, self.speed_meters_per_sec)
        )

        # Drop the piece
        self.descend_grip_ascend_chain(
            w_up=end_w_up,
            w_down=end_w_down,
            new_gripper_span=Skeleton.GRIPPER_OPEN_POSITION
        )

        # Move back to home position to end
        if home_waypoint is not None:
            rospy.loginfo("Moving arm to home position")
            self.skeleton.issue_arm_command(
                joint_vector=home_waypoint.jv,
                duration=self.std_duration
            )
        return True

    def descend_grip_ascend_chain(
        self,
        w_up: Waypoint,
        w_down: Waypoint,
        new_gripper_span: float
    ) -> bool:
        # Descend to down position
        self.skeleton.issue_arm_command(
            joint_vector=w_down.jv,
            duration=self.std_duration
        )

        # Issue new gripper span
        self.skeleton.issue_gripper_command(
            gripper_span=new_gripper_span,
            speed=0.05
        )

        # Ascend back to up position
        self.skeleton.issue_arm_command(
            joint_vector=w_up.jv,
            duration=self.std_duration
        )

        return True

    def compute_duration(self, start: Waypoint, end: Waypoint, speed: float) -> float:
        return np.clip(
            a=np.linalg.norm(start.pos - end.pos) / speed,
            a_min=self.min_duration,
            a_max=self.max_duration
        )

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
