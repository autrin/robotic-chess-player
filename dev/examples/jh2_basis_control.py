#!/usr/bin/env python3
import rospy

from jh2.orchestrator.robotics import Armature, Effector, JointVector


def main():
    robot = Armature()
    gripper = Effector(robot)

    # Move to reset position (wrist down, parallel to ground)
    reset_joint_vec = JointVector(
        0.0,     # shoulder_pan
        -1.57,   # shoulder_lift
        1.57,    # elbow
        -1.57,   # wrist_1
        -1.57,   # wrist_2
        0.0      # wrist_3
    )
    robot.apply_joint_vector(reset_joint_vec)

    # First pick and place
    pick_and_place(robot, gripper, pick_x=0.5, pick_y=0.2, drop_x=0.2, drop_y=0.5)

    # Second pick and place
    pick_and_place(robot, gripper, pick_x=0.6, pick_y=0.3, drop_x=0.1, drop_y=0.4)

    rospy.signal_shutdown("Task Complete")


def pick_and_place(robot: Armature, gripper: Effector, pick_x, pick_y, drop_x, drop_y, move_distance=0.6):
    rospy.loginfo("Starting pick-and-place sequence...")

    # Move to pick position
    robot.move_to_position(pick_x, pick_y, robot.get_current_pose().position.z)

    # Lower to pick
    robot.move_to_position(pick_x, pick_y, robot.get_current_pose().position.z - move_distance)

    rospy.sleep(1)
    gripper.close()

    # Lift after pick
    robot.move_to_position(pick_x, pick_y, robot.get_current_pose().position.z + move_distance)

    # Move to drop position
    robot.move_to_position(drop_x, drop_y, robot.get_current_pose().position.z)

    # Lower to drop
    robot.move_to_position(drop_x, drop_y, robot.get_current_pose().position.z - move_distance)

    rospy.sleep(1)
    gripper.open()

    # Lift after drop
    robot.move_to_position(drop_x, drop_y, robot.get_current_pose().position.z + move_distance)

    rospy.loginfo("Pick-and-place complete.")


if __name__ == "__main__":
    main()
