import numpy as np
from scipy.optimize import least_squares
from typing import Optional, Union

from ._forward_kinematics import ur10e_forward_kinematics
from ._joint_vector import JointVector
from jh1.typealias import Vec3


def ur10e_inverse_kinematics(
        target_pos: Vec3,
        initial_q: Optional[Union[JointVector, np.ndarray, list]] = None,
        tol: float = 1e-5,
        max_iter: int = 200
) -> JointVector:
    """
    Numerically solves for the UR10e joint angles that place the end-effector at target_pos.

    Args:
        target_pos (Vec3): Desired EE position [x, y, z].
        initial_q (JointVector | array-like, optional): Initial guess for the 6 joint angles.
            Defaults to zeros.
        tol (float): Tolerance for convergence (both xtol and ftol).
        max_iter (int): Maximum number of function evaluations.

    Returns:
        JointVector: The solved joint angles.
    """
    # Prepare target
    p_target = np.asarray(target_pos, dtype=float)

    print(f"[ur10e_inverse_kinematics] Solving IK for {target_pos}")

    # Initial guess
    if initial_q is None:
        q0 = np.zeros(6)
    elif isinstance(initial_q, JointVector):
        q0 = initial_q.as_np()
    else:
        q0 = np.array(initial_q, dtype=float)

    # Joint limits (±2π for each joint)
    lower_bounds = np.full(6, -2 * np.pi)
    upper_bounds = np.full(6, 2 * np.pi)

    # Residual: difference between current EE position and target
    def residual(q_vec):
        ee_pos = ur10e_forward_kinematics(q_vec)[-1]
        return ee_pos - p_target

    # Solve using Levenberg–Marquardt with bounds
    result = least_squares(
        residual,
        q0,
        bounds=(lower_bounds, upper_bounds),
        xtol=tol,
        ftol=tol,
        max_nfev=max_iter
    )

    if not result.success:
        raise RuntimeError(f"IK did not converge: {result.message}")

    print(f"[ur10e_inverse_kinematics] IK solution converged!")
    return JointVector.from_list(result.x.tolist())
