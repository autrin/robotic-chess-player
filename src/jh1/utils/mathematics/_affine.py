from typing import List
from jh1.typealias import *
import numpy as np


def rot_2d_mat3(theta: float) -> Mat3x3:
    """
    Construct a 2D rotation matrix R ∈ SE(2) in homogeneous coordinates.

    Returns a 3×3 matrix in SE(2), representing a counter-clockwise rotation by `theta` radians.
    The top-left 2×2 block belongs to SO(2).
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [[c, -s, 0],
         [s, c, 0],
         [0, 0, 1]],
        dtype=np.float64
    )


def scale_2d_mat3(scale: float) -> Mat3x3:
    """
    Construct a 2D isotropic scaling matrix M ∈ GL(3) in homogeneous coordinates.
    """
    return np.array(
        [[scale, 0, 0],
         [0, scale, 0],
         [0, 0, 1]],
        dtype=np.float64
    )


def shear_2d_mat3(shear: float) -> Mat3x3:
    """
    Construct a 2D shear matrix S ∈ Aff(2) along the x-axis in homogeneous coordinates.
    """
    return np.array(
        [[1, shear, 0],
         [0, 1, 0],
         [0, 0, 1]],
        dtype=np.float64
    )


def transl_2d_mat3(tx: float, ty: float) -> Mat3x3:
    """
    Construct a 2D translation matrix in homogeneous coordinates T ∈ SE(2).
    """
    return np.array(
        [[1, 0, tx],
         [0, 1, ty],
         [0, 0, 1]],
        dtype=np.float64
    )


def shift_center_mat3(cx: float, cy: float) -> Mat3x3:
    """
    Creates a matrix M ∈ SE(2) that translates the coordinate origin to an offset center (cx, cy).
    """
    return transl_2d_mat3(-cx, -cy)


def unshift_center_mat3(cx: float, cy: float) -> Mat3x3:
    """
    Creates a matrix M ∈ SE(2) that translates the coordinate origin back from an offset center
    (cx, cy) back to (0,0).
    """
    return transl_2d_mat3(cx, cy)


def compose_mat3(matrices: List[Mat3x3]) -> Mat3x3:
    """
    Compose a sequence of 3×3 matrices.
    """
    result = np.eye(3, dtype=np.float64)
    for mat in matrices: result = result @ mat
    return result


# 3D transformations — homogeneous 4×4 matrices

def rot_roll_mat4(theta: float) -> Mat4x4:
    """
    Construct a 3D rotation matrix R ∈ SE(3) ⋉ 0, representing a rotation about the X-axis (roll)
    in homogeneous coordinates.
    """
    c, s = np.cos(theta), np.sin(theta)
    mat = np.eye(4, dtype=np.float64)
    mat[1, 1], mat[1, 2] = c, -s
    mat[2, 1], mat[2, 2] = s, c
    return mat


def rot_pitch_mat4(theta: float) -> Mat4x4:
    """
    Construct a 3D rotation matrix R ∈ SE(3) ⋉ 0, representing a rotation about the Y-axis (pitch)
    in homogeneous coordinates.
    """
    c, s = np.cos(theta), np.sin(theta)
    mat = np.eye(4, dtype=np.float64)
    mat[0, 0], mat[0, 2] = c, s
    mat[2, 0], mat[2, 2] = -s, c
    return mat


def rot_azimuth_mat4(theta: float) -> Mat4x4:
    """
    Construct a 3D rotation matrix R ∈ SO(3) ⋉ 0, representing a rotation about the Z-axis (azimuth)
    in homogeneous coordinates.
    """
    c, s = np.cos(theta), np.sin(theta)
    mat = np.eye(4, dtype=np.float64)
    mat[0, 0], mat[0, 1] = c, -s
    mat[1, 0], mat[1, 1] = s, c
    return mat


def transl_3d_mat4(tx: float, ty: float, tz: float) -> Mat4x4:
    """
    Construct a 3D translation matrix T ∈ SE(3)) in homogeneous coordinates.
    """
    mat = np.eye(4, dtype=np.float64)
    mat[:3, 3] = [tx, ty, tz]
    return mat


def se3_transform_mat4(r: Mat3x3, t: Vec3) -> Mat4x4:
    """
    Construct a full 3D rigid-body transformation matrix from rotation R and translation t.

    R must belong to SO(3), and the result is a member of SE(3):
        T = [ R | t ]
            [ 0 | 1 ]
    """
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = r
    mat[:3, 3] = t
    return mat
