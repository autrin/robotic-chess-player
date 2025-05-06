import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

from jh1.robotics import Armature
from jh1.robotics.kinematics import JointVector

joint_vectors = [
    JointVector.from_list([2.2015, -1.7744, 1.1871, -2.0474, -1.5897, 2.0208]),
    JointVector.from_list([1.5139, -1.1724, 1.2701, -1.9292, -1.5697, 2.0213]),
    JointVector.from_list([1.7860, -1.0432, 1.3300, -2.3513, -1.5691, 2.0217])
]

fixed_mask = [1, 0, 0, 1, 0, 1, 0, 0, 0]
ctrl_indices = [i for i, m in enumerate(fixed_mask) if m == 0]

steps_per_segment = 20
interval_ms = 150

all_q = []
for a, b in zip(joint_vectors[:-1], joint_vectors[1:]):
    q1 = a.as_np()
    q2 = b.as_np()
    for t in np.linspace(0, 1, steps_per_segment, endpoint=False):
        all_q.append((1 - t) * q1 + t * q2)
all_q.append(joint_vectors[-1].as_np())

positions_list = [
    Armature.forward_kinematics(JointVector.from_list(q.tolist()))
    for q in all_q
]

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], '-o', linewidth=3, markersize=7)

texts = [
    ax.text(0, 0, 0, "", fontsize=8, color='k')
    for _ in JointVector.JOINT_LABELS
]

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_box_aspect([1, 1, 1])
reach = 1.4
ax.set_xlim(-reach, reach)
ax.set_ylim(-reach, reach)
ax.set_zlim(0, reach)
trail_lines = []

def init():
    line.set_data([], [])
    line.set_3d_properties([])
    for text in texts:
        text.set_text("")
    for tline in trail_lines:
        tline.remove()
    trail_lines.clear()
    return (line, *texts)

def update(frame):
    q = all_q[frame]
    pos = positions_list[frame]

    line.set_data(pos[:, 0], pos[:, 1])
    line.set_3d_properties(pos[:, 2])

    for tline in trail_lines:
        tline.remove()
    trail_lines.clear()

    for i in range(frame):
        prev_pos = positions_list[i]
        trail = ax.plot(
            prev_pos[:, 0], prev_pos[:, 1], prev_pos[:, 2],
            color="tab:purple", linewidth=1, alpha=0.3
        )[0]
        trail_lines.append(trail)

    for idx, joint_idx in enumerate(ctrl_indices):
        x, y, z = pos[joint_idx]
        name = JointVector.JOINT_LABELS[idx]
        texts[idx] = ax.text(x, y, z, f"{name}: {q[idx]:.4f}", fontsize=8, color='k')

    return (line, *texts, *trail_lines)

ani = FuncAnimation(
    fig, update, frames=len(positions_list),
    init_func=init, interval=interval_ms, blit=True
)

plt.show()