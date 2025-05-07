import numpy as np
from scipy.optimize import least_squares
from typing import Optional, Union, Sequence

from ._forward_kinematics import ur10e_forward_kinematics
from ._joint_vector import JointVector
from jh1.typealias import Vec3

def ur10e_partial_inverse_kinematics(
    target_pos: Vec3,
    initial_q_hat: Optional[Union[JointVector, np.ndarray, list]] = None,
    bone_joints_lower_bounds: Optional[Union[Sequence[float], np.ndarray]] = None,
    bone_joints_upper_bounds: Optional[Union[Sequence[float], np.ndarray]] = None,
    tol: float = 1e-5,
    max_iter: int = 10000
) -> JointVector:
    """
    Numerically solves for the UR10e joint angles that place the end-effector at target_pos.
    """
    # Prepare target
    p_target = np.asarray(target_pos, dtype=float)


    # print(f"[ur10e_partial_inverse_kinematics] Solving IK for {target_pos}")

    # Initial guess
    if initial_q_hat is None:
        q0 = np.zeros(3)
    elif isinstance(initial_q_hat, JointVector):
        q0 = np.array([initial_q_hat.shoulder_pan, initial_q_hat.shoulder_lift, initial_q_hat.elbow])
    else:
        q0 = np.array(initial_q_hat, dtype=float)

    # Joint limits
    if bone_joints_lower_bounds is None:
        lower_bounds = np.full(3, -2 * np.pi)
    else:
        lower_bounds = np.array(bone_joints_lower_bounds, dtype=float)
    if bone_joints_upper_bounds is None:
        upper_bounds = np.full(3,  2 * np.pi)
    else:
        upper_bounds = np.array(bone_joints_upper_bounds, dtype=float)


    def residual(q_hat):
        q = JointVector.from_upper_joints(*q_hat).as_np()
        ee_fwd_pos = ur10e_forward_kinematics(q)[-1]
        return ee_fwd_pos - p_target

    # Solve using TRF with bounds
    result = least_squares(
        residual,
        q0,
        bounds=(lower_bounds, upper_bounds),
        xtol=tol,
        ftol=tol,
        max_nfev=max_iter,
    )

    if not result.success:
        raise RuntimeError(f"IK did not converge: {result.message}")


    # print(f"[ur10e_partial_inverse_kinematics] IK solution converged! ({result.cost=:.4e}, {result.nfev=})")
    return JointVector.from_upper_joints(*result.x.tolist())