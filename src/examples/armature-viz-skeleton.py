import matplotlib.pyplot as plt

from jh1.robotics import Armature
from jh1.robotics.kinematics import JointVector
from jh1.utils.mathematics.affine import *
from jh1.robotics.kinematics._armature_defs import *

import numpy as np

q = [2.2015, -1.7744, 1.1871, -2.0474, -1.5897, 2.0208]
# q = [1.5139, -1.1724, 1.2701, -1.9292, -1.5697, 2.0213]
positions = Armature.forward_kinematics(Armature.adaptive_leveling(JointVector.from_list(q)))

# Mask indicating fixed (1) vs controllable (0) joints/frames
fixed_mask = [1, 0, 0, 1, 0, 1, 0, 0, 0]
ctrl_indices = [i for i, m in enumerate(fixed_mask) if m == 0]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], '-o')

for j, name in enumerate(JointVector.JOINT_LABELS):
    idx = ctrl_indices[j]
    x, y, z = positions[idx]
    ax.text(x, y, z, f"{name}: {q[j]:.4f}", fontsize=10, color='blue')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_box_aspect([1, 1, 1])
plt.show()
