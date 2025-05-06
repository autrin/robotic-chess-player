from typing import Dict

import numpy as np

from jh1.robotics import Skeleton
from jh1.robotics.kinematics import JointVector
from ._waypoint import Waypoint

## --- EDIT THESE ---
up_height_meters = 0.08
home_jv = JointVector.from_topic([2.449476, -1.842133, 1.006606, 0, 0, 0]).adaptive_leveling()
discard_jv = JointVector.from_topic([3.449476, -1.642133, 1.006606, 0, 0, 0]).adaptive_leveling()

a8 = Skeleton.forward_kinematics(JointVector.from_topic(
    [1.9908550421344202, -1.0797357720187684, 1.2676620483398438, -2.4606195888915003,
     -1.6312678495990198, 1.6715844869613647]
))[-1]
h8 = Skeleton.forward_kinematics(JointVector.from_topic(
    [1.0309518019305628, -0.5775613945773621, 1.2293174266815186, -2.0083195171751917,
     -1.6719935576068323, 1.6715705394744873]
))[-1]
h1 = Skeleton.forward_kinematics(JointVector.from_topic(
    [1.0655153433429163, -0.5972040456584473, 0.8859420418739319, -2.010952135125631,
     -1.5819533506976526, 1.3885889053344727]
))[-1]

UP_LABEL_SUFFIX = "_up"

## --- GENERATE BASIS VECTORS AND WAYPOINTS ---
WAYPOINT_TABLE: Dict[str, Waypoint] = {}

HOME_WAYPOINT = WAYPOINT_TABLE["home"] = Waypoint(
    label="home",
    pos=Skeleton.forward_kinematics(home_jv)[-1],
    jv=home_jv
)

DISCARD_WAYPOINT = WAYPOINT_TABLE["discard"] = Waypoint(
    label="discard",
    pos=Skeleton.forward_kinematics(discard_jv)[-1],
    jv=discard_jv
)

discard_up_pos = DISCARD_WAYPOINT.pos + np.array([0, 0, up_height_meters])
DISCARD_UP_WAYPOINT = WAYPOINT_TABLE["discard_up"] = Waypoint(
    label="discard_up",
    pos=discard_up_pos,
    jv=Skeleton.partial_inverse_kinematics(discard_up_pos)
)

x_hat = (h8 - a8) / 7
y_hat = (h1 - h8) / 7

board = {}
files = "abcdefgh"
for i, file in enumerate(files):
    for rank in range(1, 9):
        pos = a8 + i * x_hat + (8 - rank) * y_hat
        board[f"{file}{rank}"] = pos

for k, v in board.items():
    print(f"[calibrate_basis] Generating IK waypoints for square {k}")
    WAYPOINT_TABLE[k] = Waypoint(
        label=k,
        pos=v,
        jv=Skeleton.partial_inverse_kinematics(v)
    )

    v_up = v + np.array([0, 0, up_height_meters])
    WAYPOINT_TABLE[k + UP_LABEL_SUFFIX] = Waypoint(
        label=k,
        pos=v_up,
        jv=Skeleton.partial_inverse_kinematics(v_up)
    )
