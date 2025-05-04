#!/usr/bin/env python3

"""
Description: Sample test for using the RobotUR10eGripper class

Author: Ling Tang
Date: 2025-02-16
Version: 1.0
"""

from robot_ur10e_gripper import RobotUR10eGripper
import rospy

if __name__ == '__main__':
    robot =  RobotUR10eGripper(True)
    rate = rospy.Rate(1)

    # robot._command_gripper(0.01)

    # robot._command_ur10e([ -2.62,-1.71, 2.07, -0.76, 0.0, 1.0], 10)

    # robot.command_robot([-2.62,-1.71, 2.07, -0.76, 0.0, 1.0, 0.05], 5.0)
    robot.command_robot([-2.62,-1.71, 2.07, -0.56, 0.1, 0.5, 0.02], 5.0)

    print("move done")

    while not rospy.is_shutdown():
        print(robot.get_info())
        rate.sleep()