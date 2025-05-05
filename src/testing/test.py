#!/usr/bin/env python3

"""
Description: Sample test for using the RobotUR10eGripper class

Author: Ling Tang
Date: 2025-02-16
Version: 1.0
"""

from jh1.physical.robotics.robot_ur10e_gripper import RobotUR10eGripper
import rospy

if __name__ == '__main__':
    robot =  RobotUR10eGripper(True)
    rate = rospy.Rate(1)

    # robot._command_gripper(0.01)

    # robot._command_ur10e([ -2.62,-1.71, 2.07, -0.76, 0.0, 1.0], 10)

    # robot.command_robot([-2.62,-1.71, 2.07, -0.76, 0.0, 1.0, 0.05], 5.0)
    # robot.command_robot([-2.62,-1.71, 2.07, -0.56, 0.1, 0.5, 0.02], 5.0)
    rospy.loginfo("\n p1")
    robot.command_robot([2.2015607992755335, -1.7744752369322718, 1.1870899200439453, -2.0474611721434535, -1.5897491613971155, 2.020841360092163, 0.0], 5.0)
    rospy.loginfo("\n p2")
    robot.command_robot([1.5139759222613733, -1.1724217695048829, 1.270115613937378, -1.9291945896544398, -1.569782559071676, 2.0213046073913574, 0.90], 5.0) 
    rospy.loginfo("\n p3")
    robot.command_robot([1.7860372702227991, -1.0432136815837403, 1.330017328262329, -2.351302763024801, -1.5691269079791468, 2.0217325687408447, 0.05], 5.0)
    print("move done")

    while not rospy.is_shutdown():
        print(robot.get_info())
        rate.sleep()