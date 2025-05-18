from typing import List, Union
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


def shift_origin_mat3(cx: float, cy: float) -> Mat3x3:
    """
    Creates a matrix M ∈ SE(2) that translates the coordinate origin to an offset center (cx, cy).
    """
    return transl_2d_mat3(-cx, -cy)


def unshift_origin_mat3(cx: float, cy: float) -> Mat3x3:
    """
    Creates a matrix M ∈ SE(2) that translates the coordinate origin back from an offset center
    (cx, cy) back to (0,0).
    """
    return transl_2d_mat3(cx, cy)


def aff2_mat3(theta: float, scale: Vec2, shear: float, translation: Vec2) -> Mat3x3:
    """
    Build a 3x3 affine transform matrix M ∈ Aff(2) in 2D (homogeneous) from rotation, scale, shear,
    and translation.
    """
    c, s = np.cos(theta), np.sin(theta)
    r = np.array([[c, -s],
                  [s, c]])

    sh = np.array([[1, shear],
                   [0, 1]])

    rs = r @ sh * scale[np.newaxis, :]

    mat = np.eye(3)
    mat[:2, :2] = rs
    mat[:2, 2] = translation
    return mat


def compose_mat3(matrices: List[Mat3x3]) -> Mat3x3:
    """
    Compose a sequence of 3×3 matrices.
    """
    result = matrices[0]
    for mat in matrices[1:]: result = result @ mat
    return result


# 3D rotations - SO(3) matrices

def rot_roll_so3(theta: float) -> Mat3x3:
    """
    Construct a 3D rotation matrix R ∈ SO(3) representing a rotation about the X-axis (roll).

    Args:
        theta (float): Rotation angle in radians.

    Returns:
        Mat3x3: 3×3 rotation matrix for X-axis rotation.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ], dtype=np.float64)


def rot_pitch_so3(theta: float) -> Mat3x3:
    """
    Construct a 3D rotation matrix R ∈ SO(3) representing a rotation about the Y-axis (pitch).

    Args:
        theta (float): Rotation angle in radians.

    Returns:
        Mat3x3: 3×3 rotation matrix for Y-axis rotation.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ], dtype=np.float64)


def rot_azimuth_so3(theta: float) -> Mat3x3:
    """
    Construct a 3D rotation matrix R ∈ SO(3) representing a rotation about the Z-axis (azimuth).

    Args:
        theta (float): Rotation angle in radians.

    Returns:
        Mat3x3: 3×3 rotation matrix for Z-axis rotation.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ], dtype=np.float64)


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


def se3_rigid_mat4(r: Mat3x3, t: Union[Vec3, List[float]]) -> Mat4x4:
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


def aff3_mat4(r: Mat3x3, t: Vec3, s: Vec3) -> Mat4x4:
    """
    Construct a 4×4 affine transformation matrix from rotation R, translation t, and scaling s.

    - R: 3×3 rotation matrix (assumed to be in SO(3))
    - t: 3D translation vector
    - s: 3D scaling factors for x, y, z axes

    The resulting matrix is in Aff(3):
        T = [ R * diag(s) | t ]
            [     0       | 1 ]
    """
    mat = np.eye(4, dtype=np.float64)
    scaled_r = r * s[np.newaxis, :]  # Element-wise scale columns of R
    mat[:3, :3] = scaled_r
    mat[:3, 3] = t
    return mat
