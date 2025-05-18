from typing import Dict

import numpy as np

from jh1.robotics import Skeleton
from jh1.robotics.kinematics import JointVector
from ._waypoint import Waypoint

## --- EDIT THESE ---
STANDARD_UP_HEIGHT = 0.08
DROP_HEIGHT = 0.005
home_jv = JointVector.from_topic([2.4494, -1.8421, 1.012, 0, 0, 0]).adaptive_leveling()
discard_jv = JointVector.from_topic([1.5463, -0.9344, 0.4444, 0, 0, 0]).adaptive_leveling()

a1_jv = [1.8095367590533655, -0.8377025884440918, 0.9908225536346436, -2.557741781274313,
         -1.5800336042987269, -0.311380688344137]
h1_jv = [0.5234759489642542, -0.2371109288981934, 1.0894415378570557, -1.8760134182372035,
         -1.550518814717428, -0.19458419481386358]
h8_jv = [0.37623769441713506, -0.1649351877025147, 1.410502552986145, -1.811185976068014,
         -1.5535910765277308, 0.14262129366397858]
a8_jv = [1.7017152945147913, -0.7957399052432557, 1.4398081302642822, -2.4622489414610804,
         -1.5409234205829065, 0.11539431661367416]

a1 = Skeleton.forward_kinematics(JointVector.from_topic(a1_jv[:6]))[-1]
h1 = Skeleton.forward_kinematics(JointVector.from_topic(h1_jv[:6]))[-1]
h8 = Skeleton.forward_kinematics(JointVector.from_topic(h8_jv[:6]))[-1]
a8 = Skeleton.forward_kinematics(JointVector.from_topic(a8_jv[:6]))[-1]

# a8 = Skeleton.forward_kinematics(JointVector.from_topic(
#     [1.9908550421344202, -1.0797357720187684, 1.2676620483398438, -2.4606195888915003,
#      -1.6312678495990198, 1.6715844869613647]
# ))[-1]
# h8 = Skeleton.forward_kinematics(JointVector.from_topic(
#     [1.0309518019305628, -0.5775613945773621, 1.2293174266815186, -2.0083195171751917,
#      -1.6719935576068323, 1.6715705394744873]
# ))[-1]
# h1 = Skeleton.forward_kinematics(JointVector.from_topic(
#     [1.0655153433429163, -0.5972040456584473, 0.8859420418739319, -2.010952135125631,
#      -1.5819533506976526, 1.3885889053344727]
# ))[-1]

UP_LABEL_SUFFIX = "_up"
DROP_LABEL_SUFFIX = "_drop"

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

discard_up_pos = DISCARD_WAYPOINT.pos + np.array([0, 0, STANDARD_UP_HEIGHT])
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

    v_up = v + np.array([0, 0, STANDARD_UP_HEIGHT])
    WAYPOINT_TABLE[k + UP_LABEL_SUFFIX] = Waypoint(
        label=k + UP_LABEL_SUFFIX,
        pos=v_up,
        jv=Skeleton.partial_inverse_kinematics(v_up)
    )

    v_drop = v + np.array([0, 0, DROP_HEIGHT])
    WAYPOINT_TABLE[k + DROP_LABEL_SUFFIX] = Waypoint(
        label=k + DROP_LABEL_SUFFIX,
        pos=v_drop,
        jv=Skeleton.partial_inverse_kinematics(v_drop)
    )
