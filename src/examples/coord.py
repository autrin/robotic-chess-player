#!/usr/bin/env python3
from jh1.robotics.kinematics import SQUARE_IK_LOOKUP
from jh1.robotics.robot_ur10e_gripper import RobotUR10eGripper
import rospy

if __name__ == '__main__':
    robot = RobotUR10eGripper(True)
    rate = rospy.Rate(1)

    start_square = "a1"
    end_square = "h8"

    rospy.loginfo("\n Start from home")
    robot.command_robot(SQUARE_IK_LOOKUP["home"].as_command(0.9), 5.0)

    rospy.loginfo(f"\n Go to start square ({start_square})")
    robot.command_robot(SQUARE_IK_LOOKUP[start_square].as_command(0.9), 5.0)

    rospy.loginfo(f"\n Close gripper at ({start_square})")
    robot.command_robot(SQUARE_IK_LOOKUP[start_square].as_command(0.05), 5.0)

    rospy.loginfo(f"\n Go to end square ({end_square})")
    robot.command_robot(SQUARE_IK_LOOKUP[end_square].as_command(0.05), 5.0)

    rospy.loginfo(f"\n Release gripper at ({end_square})")
    robot.command_robot(SQUARE_IK_LOOKUP[end_square].as_command(0.9), 5.0)
    print("move done")

    while not rospy.is_shutdown():
        print(robot.get_info())
        rate.sleep()
