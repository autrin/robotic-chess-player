# from .robot_ur10e_gripper import RobotUR10eGripper
from jh1.robotics.kinematics import *
from jh1.typealias import *

_STANDARD_INITIAL_GUESS = np.array([1.0066, -1.8421, 2.445])
_EPSILON = 1e-3


class Armature:
    """
    A wrapper around the base RobotUR10eGripper class
    """

    def __init__(self, robot: 'RobotUR10eGripper'):
        self.robot: 'RobotUR10eGripper' = robot

    def issue_aggregated_command(self, joint_vector: JointVector, gripper_span: float,
                                 duration: float) -> bool:
        return self.robot.command_robot(joint_vector.as_aggregated_command(gripper_span), duration)

    def issue_arm_command(self, joint_vector: JointVector, duration: float) -> bool:
        # noinspection PyProtectedMember
        return self.robot._command_ur10e(joint_vector.as_command(), duration)

    def issue_gripper_command(self, gripper_span: float, speed=0.05, force=0) -> bool:
        # noinspection PyProtectedMember
        return self.robot._command_gripper(gripper_span, speed, force)

    @staticmethod
    def forward_kinematics(q: JointVector) -> NDArray[Vec3]:
        return ur10e_forward_kinematics(q.as_np())

    @staticmethod
    def inverse_kinematics(target: Vec3) -> JointVector:
        _2pi = 2 * np.pi
        _pi_over_2 = np.pi / 2

        return ur10e_inverse_kinematics(
            target,
            initial_q=JointVector.from_topic(
                [1.9908, -1.0797, 1.2676, -2.4606, -_pi_over_2, _pi_over_2]
            ),
            joint_lower_bounds=[
                -_2pi, -_2pi, -_2pi,
                -_2pi, -_pi_over_2 - _EPSILON, _pi_over_2 - _EPSILON
            ],
            joint_upper_bounds=[
                _2pi, _2pi, _2pi,
                _2pi, -_pi_over_2 + _EPSILON, _pi_over_2 + _EPSILON
            ]
        )

    @staticmethod
    def adaptive_inverse_kinematics(
        target: Vec3,
        initial_q_hat: Optional[Union[JointVector, np.ndarray, list]] = _STANDARD_INITIAL_GUESS
    ) -> JointVector:
        _2pi = 2 * np.pi
        return ur10e_adaptive_inverse_kinematics(
            target,
            initial_q_hat,
            bone_joints_lower_bounds=[-_2pi, -np.pi, -_EPSILON],
            bone_joints_upper_bounds=[_2pi, _EPSILON, np.pi]
        )

    GRIPPER_OPEN_POSITION = 0.025
    GRIPPER_CLOSED_POSITION = 0.05
