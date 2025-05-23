# from .robot_ur10e_gripper import RobotUR10eGripper
from jh1.robotics.kinematics import *
from jh1.typealias import *

_STANDARD_INITIAL_GUESS = np.array([1.0066, -1.8421, 2.445])
_EPSILON = 1e-3


class Skeleton:
    """
    A wrapper around the base RobotUR10eGripper class
    """

    def __init__(self, robot: Optional['RobotUR10eGripper'] = None):
        self.robot: 'RobotUR10eGripper' = robot  # robot = None is understood to be testing mode
        self.configuration_vector: Optional[JointVector] = None

    def issue_aggregated_command(
        self, joint_vector: JointVector, gripper_span: float, duration: float
    ) -> bool:
        if self.robot is None:
            return Skeleton.none_type_robot_print()
        return self.robot.command_robot(joint_vector.as_aggregated_command(gripper_span), duration)

    def issue_arm_command(self, joint_vector: JointVector, duration: float) -> bool:
        if self.robot is None:
            return Skeleton.none_type_robot_print()
        # noinspection PyProtectedMember
        return self.robot._command_ur10e(joint_vector.as_command(), duration)

    def issue_gripper_command(self, gripper_span: float, speed=0.05, force=0.01) -> bool:
        if self.robot is None:
            return Skeleton.none_type_robot_print()
        # noinspection PyProtectedMember
        return self.robot._command_gripper(gripper_span, speed, force)

    @staticmethod
    def none_type_robot_print():
        print("[Armature] Robot is None, so no command has been carried out.")
        return True

    @staticmethod
    def forward_kinematics(q: JointVector) -> NDArray[Vec3]:
        return ur10e_forward_kinematics(q.as_np())

    @staticmethod
    def partial_inverse_kinematics(
        target: Vec3,
        initial_q_hat: Optional[Union[JointVector, np.ndarray, list]] = _STANDARD_INITIAL_GUESS
    ) -> JointVector:
        _2pi = 2 * np.pi
        return ur10e_partial_inverse_kinematics(
            target,
            initial_q_hat,
            bone_joints_lower_bounds=[-_2pi, -np.pi, -_EPSILON],
            bone_joints_upper_bounds=[_2pi, _EPSILON, np.pi]
        )

    GRIPPER_OPEN_POSITION = 0.90
    GRIPPER_HALF_OPEN_POSITION = 0.05
    GRIPPER_CLOSED_POSITION = 0.0385
