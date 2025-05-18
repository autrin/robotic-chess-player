import matplotlib.pyplot as plt

from jh1.robotics import Skeleton
from jh1.robotics.kinematics import JointVector
from jh1.utils.mathematics.affine import *
from jh1.robotics.kinematics._armature_defs import *

import numpy as np

q = [2.2015, -1.7744, 1.2579,
     -2.0474, -1.5897, 2.0208]
# q = [1.5139, -1.1724, 1.2701, -1.9292, -1.5697, 2.0213]
q = JointVector.from_list(q).adaptive_leveling().as_list()
positions = Skeleton.forward_kinematics(JointVector.from_list(q))

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



x_limits = ax.get_xlim3d()
y_limits = ax.get_ylim3d()
z_limits = ax.get_zlim3d()
x_range = x_limits[1] - x_limits[0]
y_range = y_limits[1] - y_limits[0]
z_range = z_limits[1] - z_limits[0]
max_range = max(x_range, y_range, z_range) / 2.0
x_mid = sum(x_limits) / 2.0
y_mid = sum(y_limits) / 2.0
z_mid = sum(z_limits) / 2.0
ax.set_xlim3d(x_mid - max_range, x_mid + max_range)
ax.set_ylim3d(y_mid - max_range, y_mid + max_range)
ax.set_zlim3d(z_mid - max_range, z_mid + max_range)

plt.show()
plt.show()
