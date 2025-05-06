import numpy as np

from jh1.utils.mathematics.affine import se3_rigid_mat4, rot_azimuth_so3, rot_pitch_so3
from ._constants import (
    UR10E_BASE_ROT_R,
    UR10E_R_Y_90,
    UR10E_ARMATURE_D1,
    UR10E_ARMATURE_SHOULDER_OFFSET,
    UR10E_ARMATURE_A2,
    UR10E_ARMATURE_ELBOW_OFFSET,
    UR10E_ARMATURE_A3,
    UR10E_ARMATURE_D4,
    UR10E_ARMATURE_D5,
    UR10E_ARMATURE_EE_OFFSET
)


def ur10e_forward_kinematics(q):
    """
    Compute the 3D positions of each joint frame (and EE site)
    for the UR10e skeleton.

    Args:
        q (array-like, shape (6,)): [q1, q2, q3, q4, q5, q6]

    Returns:
        positions (ndarray, shape (8, 3)):
            x,y,z of:
            [base, shoulder, upper_arm, forearm_elbow,
             forearm_end, wrist1, wrist2, wrist3, ee_site]
    """
    fk_mat = np.eye(4)
    positions = [fk_mat[:3, 3].copy()]  # base origin

    # 1) Base fixed rotation
    fk_mat = fk_mat @ se3_rigid_mat4(UR10E_BASE_ROT_R, np.zeros(3))

    # 2) Shoulder link offset
    fk_mat = fk_mat @ se3_rigid_mat4(np.eye(3), UR10E_ARMATURE_D1)
    positions.append(fk_mat[:3, 3].copy())

    # 3) Shoulder pan joint
    fk_mat = fk_mat @ se3_rigid_mat4(rot_azimuth_so3(q[0]), np.zeros(3))

    # 4) Upper arm link offset
    fk_mat = fk_mat @ se3_rigid_mat4(UR10E_R_Y_90, UR10E_ARMATURE_SHOULDER_OFFSET)
    positions.append(fk_mat[:3, 3].copy())

    # 5) Shoulder lift joint
    fk_mat = fk_mat @ se3_rigid_mat4(rot_pitch_so3(q[1]), np.zeros(3))

    # 6) Main forearm link
    fk_mat = fk_mat @ se3_rigid_mat4(np.eye(3), UR10E_ARMATURE_A2)
    positions.append(fk_mat[:3, 3].copy())

    # 7) Elbow-offset bone
    fk_mat = fk_mat @ se3_rigid_mat4(np.eye(3), UR10E_ARMATURE_ELBOW_OFFSET)
    positions.append(fk_mat[:3, 3].copy())

    # 8) Elbow joint
    fk_mat = fk_mat @ se3_rigid_mat4(rot_pitch_so3(q[2]), np.zeros(3))

    # 9) Wrist1 link offset
    fk_mat = fk_mat @ se3_rigid_mat4(UR10E_R_Y_90, UR10E_ARMATURE_A3)
    positions.append(fk_mat[:3, 3].copy())

    # 10) Wrist1 joint
    fk_mat = fk_mat @ se3_rigid_mat4(rot_pitch_so3(q[3]), np.zeros(3))

    # 11) Wrist2 link offset
    fk_mat = fk_mat @ se3_rigid_mat4(np.eye(3), UR10E_ARMATURE_D4)
    positions.append(fk_mat[:3, 3].copy())

    # 12) Wrist2 joint
    fk_mat = fk_mat @ se3_rigid_mat4(rot_azimuth_so3(q[4]), np.zeros(3))

    # 13) Wrist3 link offset
    fk_mat = fk_mat @ se3_rigid_mat4(np.eye(3), UR10E_ARMATURE_D5)
    positions.append(fk_mat[:3, 3].copy())

    # 14) Wrist3 joint
    fk_mat = fk_mat @ se3_rigid_mat4(rot_pitch_so3(q[5]), np.zeros(3))

    # 15) End-effector site
    fk_mat = fk_mat @ se3_rigid_mat4(np.eye(3), UR10E_ARMATURE_EE_OFFSET)
    positions.append(fk_mat[:3, 3].copy())

    return np.array(positions)
