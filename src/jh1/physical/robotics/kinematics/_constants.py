import numpy as np
from jh1.utils.mathematics.affine import (
    rot_pitch_so3,
    rot_azimuth_so3
)

# These measurements can be found in the URDF or MJCF file.

# @formatter:off
# ———————— Constants ————————
# Fixed rotations
UR10E_BASE_ROT_R       = rot_azimuth_so3(np.pi)        # 180° about Z
UR10E_R_Y_90           = rot_pitch_so3(np.pi / 2)      # +90° about Y

# Link offsets (in meters)
UR10E_ARMATURE_D1               = np.array([0.0, 0.0, 0.181])   # base → shoulder_link
UR10E_ARMATURE_SHOULDER_OFFSET  = np.array([0.0, 0.0, 0.176])   # shoulder_link → upper_arm_link
UR10E_ARMATURE_A2               = np.array([0.0, 0.0, 0.613])   # upper_arm_link → forearm_link
UR10E_ARMATURE_ELBOW_OFFSET     = np.array([0.0, -0.137, 0.0])  # small link at elbow
UR10E_ARMATURE_A3               = np.array([0.0, 0.0, 0.571])   # forearm_link → wrist_1_link
UR10E_ARMATURE_D4               = np.array([0.0, 0.0, 0.135])   # wrist_1_link → wrist_2_link
UR10E_ARMATURE_D5               = np.array([0.0, 0.0, 0.120])   # wrist_2_link → wrist_3_link
UR10E_ARMATURE_EE_OFFSET        = np.array([0.0, 0.1, 0.0])     # wrist_3_link → attachment_site
# @formatter:on