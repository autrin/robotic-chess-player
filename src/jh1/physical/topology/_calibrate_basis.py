from typing import Dict

import numpy as np

from jh1.robotics import Armature
from jh1.robotics.kinematics import JointVector
from ._waypoint import Waypoint

## --- EDIT THESE ---
up_height_meters = 0.08
home_jv = JointVector.from_topic([2.449476, -1.842133, 1.006606, 0, 0, 0]).adaptive_leveling()
a8 = Armature.forward_kinematics(JointVector.from_topic(
    [1.9908550421344202, -1.0797357720187684, 1.2676620483398438, -2.4606195888915003,
     -1.6312678495990198, 1.6715844869613647]
))[-1]
h8 = Armature.forward_kinematics(JointVector.from_topic(
    [1.0309518019305628, -0.5775613945773621, 1.2293174266815186, -2.0083195171751917,
     -1.6719935576068323, 1.6715705394744873]
))[-1]
h1 = Armature.forward_kinematics(JointVector.from_topic(
    [1.0655153433429163, -0.5972040456584473, 0.8859420418739319, -2.010952135125631,
     -1.5819533506976526, 1.3885889053344727]
))[-1]

UP_LABEL_SUFFIX = "_up"

## --- GENERATE BASIS VECTORS AND WAYPOINTS ---

x_dir = (h8 - a8) / 7
y_dir = (h1 - h8) / 7

z_base = (a8[2] + h8[2] + h1[2]) / 3

board = {}
files = "abcdefgh"
for i, file in enumerate(files):
    for rank in range(1, 9):
        pos = a8 + i * x_dir + (8 - rank) * y_dir
        board[f"{file}{rank}"] = pos

SQUARE_IK_LOOKUP: Dict[str, Waypoint] = {}

for k, v in board.items():
    print(f"[calibrate_basis] Generating IK waypoints for square {k}")
    SQUARE_IK_LOOKUP[k] = Waypoint(
        label=k,
        pos=v,
        angles_ik=Armature.adaptive_inverse_kinematics(v)
    )

    v_up = v + np.array([0, 0, up_height_meters])
    SQUARE_IK_LOOKUP[k + UP_LABEL_SUFFIX] = Waypoint(
        label=k,
        pos=v_up,
        angles_ik=Armature.adaptive_inverse_kinematics(v_up)
    )

HOME_WAYPOINT = SQUARE_IK_LOOKUP["home"] = Waypoint(
    label="home",
    pos=Armature.forward_kinematics(home_jv),
    angles_ik=home_jv
)
