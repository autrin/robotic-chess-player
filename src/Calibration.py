from jh1.physical.robotics.robot_ur10e_gripper import RobotUR10eGripper
import rospy


class Calibration:
    """
        Assign the insatnce of RobotUr10eGripper to Calibration::robot
        The calibration_points and robot variables are static in case we need to
        access the calibration points elsewhere
    """
    calibration_points = {"corner1": None, "corner2": None, "corner3": None, "corner4": None}
    robot: RobotUR10eGripper = None

    def init(self):
        pass

    @staticmethod
    def calibrate():
        """
        This is a call back method.
        Call this for calibration.
        It will prompt the user with one target point at a time.
        User needs to press enter to save the joint position of each target point.
        """
        try:
            for corner in Calibration.calibration_points.keys():
                print("move the robot to " + corner + " and press enter")
                input()
                Calibration.calibration_points[corner] = Calibration.robot.get_joint_pos()

            for corner in Calibration.calibration_points.keys():
                if Calibration.calibration_points[corner] is None:
                    raise Exception

        except Exception as e:
            rospy.loginfo("Calibration failed!")
            pass
        return

    @staticmethod
    def get_calibrated_pos_of_corners():
        """
        returns None if at least one joint position is missing
        """
        missing = False
        for corner in Calibration.calibration_points.keys():
            if Calibration.calibration_points[corner] is None:
                missing = True
                break

        if missing:
            return None
        else:
            return Calibration.calibration_points
