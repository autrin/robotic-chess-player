import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

from jh1.physical.topology import SQUARE_IK_LOOKUP
from jh1.robotics import Armature
from jh1.robotics.kinematics import JointVector


def animate_joint_vectors(joint_vectors, steps_per_segment=20, interval_ms=150, fixed_mask=None):
    if fixed_mask is None:
        fixed_mask = [0] * len(
            Armature.forward_kinematics(joint_vectors[0]))  # Assume all controllable

    ctrl_indices = [i for i, m in enumerate(fixed_mask) if m == 0]

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
    texts = [ax.text(0, 0, 0, "", fontsize=8, color='k') for _ in JointVector.JOINT_LABELS]
    trail_lines = []
    ee_trail_x, ee_trail_y, ee_trail_z = [], [], []
    ee_trail_line, = ax.plot([], [], [], color="tab:orange", linewidth=3, alpha=0.8)

    ax.set_box_aspect([1, 1, 1])
    reach = 1.4
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_zlim(0, reach)

    def init():
        plt.title("Proposed motion", fontsize=16, pad=20)
        line.set_data([], [])
        line.set_3d_properties([])
        ee_trail_line.set_data([], [])
        ee_trail_line.set_3d_properties([])
        for text in texts:
            text.set_text("")
        for tline in trail_lines:
            tline.remove()
        trail_lines.clear()
        ee_trail_x.clear()
        ee_trail_y.clear()
        ee_trail_z.clear()
        return (line, ee_trail_line, *texts)

    def update(frame):
        q = all_q[frame]
        pos = positions_list[frame]

        # Update main line
        line.set_data(pos[:, 0], pos[:, 1])
        line.set_3d_properties(pos[:, 2])

        # Phantom trail of past robot poses
        for tline in trail_lines:
            tline.remove()
        trail_lines.clear()
        for i in range(frame):
            prev_pos = positions_list[i]
            tline = ax.plot(
                prev_pos[:, 0], prev_pos[:, 1], prev_pos[:, 2],
                color="tab:purple", linewidth=1, alpha=0.3
            )[0]
            trail_lines.append(tline)

        # Update joint texts
        for idx, joint_idx in enumerate(ctrl_indices):
            x, y, z = pos[joint_idx]
            name = JointVector.JOINT_LABELS[idx]
            texts[idx] = ax.text(x, y, z, f"{name}: {q[idx]:.4f}", fontsize=8, color='k')

        # Update end-effector trail (last position)
        ee = pos[-1]
        ee_trail_x.append(ee[0])
        ee_trail_y.append(ee[1])
        ee_trail_z.append(ee[2])
        ee_trail_line.set_data(ee_trail_x, ee_trail_y)
        ee_trail_line.set_3d_properties(ee_trail_z)

        return (line, ee_trail_line, *texts, *trail_lines)

    ani = FuncAnimation(
        fig, update, frames=len(positions_list),
        init_func=init, interval=interval_ms, blit=True
    )
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.9)

    for q in joint_vectors:
        pos = Armature.forward_kinematics(q)
        ee_pos = pos[-1]  # Last frame = end-effector
        ax.scatter(
            [ee_pos[0]], [ee_pos[1]], [ee_pos[2]],
            color='tab:red', s=50, marker='X'
        )
    plt.show()


if __name__ == '__main__':
    joint_vectors = [
        SQUARE_IK_LOOKUP['home'].angles_ik,
        SQUARE_IK_LOOKUP['a1'].angles_ik,
        SQUARE_IK_LOOKUP['h1'].angles_ik,
        SQUARE_IK_LOOKUP['h8'].angles_ik,
        SQUARE_IK_LOOKUP['a8'].angles_ik,
        SQUARE_IK_LOOKUP['e2'].angles_ik,
        SQUARE_IK_LOOKUP['e4'].angles_ik,

        # JointVector.from_topic([2.113312069569723, -1.2614153188518067, 0.6471139192581177, -2.404459138909811, -1.5351746718036097, 1.085855484008789]),
        # JointVector.from_topic([1.9908550421344202, -1.0797357720187684, 1.2676620483398438, -2.4606195888915003, -1.6312678495990198, 1.6715844869613647]),
        # JointVector.from_topic([1.0309518019305628, -0.5775613945773621, 1.2293174266815186, -2.0083195171751917, -1.6719935576068323, 1.6715705394744873]),
        # JointVector.from_topic([1.0655153433429163, -0.5972040456584473, 0.8859420418739319, -2.010952135125631, -1.5819533506976526, 1.3885889053344727]),
        # JointVector.from_topic([2.4494758288012903, -1.842133184472555, 1.006606101989746, -2.13765873531484,-1.580822769795553, 1.3886008262634277])

        # JointVector.from_list([2.2015, -1.7744, 1.1871, -2.0474, -1.5897, 2.0208]),
        # # JointVector.from_list([1.5139, -1.1724, 1.2701, -1.9292, -1.5697, 2.0213]),
        # # JointVector.from_list([1.7860, -1.0432, 1.3300, -2.3513, -1.5691, 2.0217])
        # Armature.inverse_kinematics(np.array([0, -0.7, 0.4])),
        # Armature.inverse_kinematics(np.array([0, -0.2, 0.4])),
        # Armature.inverse_kinematics(np.array([0.5, -0.2, 0.4])),
        # Armature.inverse_kinematics(np.array([0.5, -0.7, 0.4]))
    ]

    fixed_mask = [1, 0, 0, 1, 0, 1, 0, 0, 0]
    animate_joint_vectors(joint_vectors, fixed_mask=fixed_mask)
