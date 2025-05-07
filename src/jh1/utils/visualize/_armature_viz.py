import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from jh1.topology import WAYPOINT_TABLE
from jh1.robotics import Skeleton
from jh1.robotics.kinematics import JointVector
from jh1.typealias import Mat4x4
from jh1.utils.mathematics.affine import aff3_mat4


def draw_chessboard(
    ax, a1, a8, h1, nx=8, ny=8,
    color1="#e1f7ec", color2="#5aad82", a1_color="#753947", alpha=0.5
):
    """
    Draw an nx×ny chessboard whose *cell centers* at a1,a8,h1,h8 are given.
    """

    # 1) Build local board axes and origin
    a1, a8, h1 = map(np.asarray, (a1, a8, h1))
    # two in‐plane vectors
    v1 = 8 / 7 * (h1 - a1)
    v2 = 8 / 7 * (a8 - a1)
    # normalized orthonormal frame
    ex = v1 / np.linalg.norm(v1)
    ey = v2 / np.linalg.norm(v2)
    ez = np.cross(ex, ey)
    ez /= np.linalg.norm(ez)
    R = np.column_stack((ex, ey, ez))

    # 2) scale from 1 local‐unit to one cell
    #    (local X, Y run 0…nx and 0…ny to span the board)
    s = np.array(
        [
            np.linalg.norm(v1) / nx,
            np.linalg.norm(v2) / ny,
            1.0
        ], dtype=np.float64
    )
    origin = a1 - (R * s[np.newaxis, :]) @ np.array([0.5, 0.5, 0.0])

    # 3) build the full 4×4
    M: Mat4x4 = aff3_mat4(R, origin, s)

    for i in range(ny):
        for j in range(nx):
            uv = np.array(
                [[j, i, 0, 1],
                 [j + 1, i, 0, 1],
                 [j + 1, i + 1, 0, 1],
                 [j, i + 1, 0, 1]]
                , dtype=np.float64
            ).T
            # map to world
            pts = (M @ uv).T[:, :3]

            square_color = a1_color if i + j == 0 else (color1 if (i + j) % 2 == 1 else color2)
            ax.add_collection3d(
                Poly3DCollection(
                    [pts],
                    facecolors=square_color,
                    alpha=alpha
                )
            )


def animate_joint_vectors(joint_vectors, steps_per_segment=8, interval_ms=100):
    fixed_mask = [1, 0, 0, 1, 0, 1, 0, 0, 0]

    ctrl_indices = [i for i, m in enumerate(fixed_mask) if m == 0]

    all_q = []
    for a, b in zip(joint_vectors[:-1], joint_vectors[1:]):
        q1 = a.as_np()
        q2 = b.as_np()
        for t in np.linspace(0, 1, steps_per_segment, endpoint=False):
            all_q.append((1 - t) * q1 + t * q2)
    all_q.append(joint_vectors[-1].as_np())

    positions_list = [
        Skeleton.forward_kinematics(JointVector.from_list(q.tolist()))
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
    ax.view_init(elev=25, azim=-150)
    ax.set_xlim(-reach * 0.6, 0)
    ax.set_ylim(-reach * 0.8, -reach * 0.2)
    ax.set_zlim(0, reach * 0.6)

    draw_chessboard(
        ax,
        a1=WAYPOINT_TABLE["a1"].pos,
        a8=WAYPOINT_TABLE["a8"].pos,
        h1=WAYPOINT_TABLE["h1"].pos,
    )

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
                color="tab:purple", linewidth=1, alpha=0.13
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
        pos = Skeleton.forward_kinematics(q)
        ee_pos = pos[-1]  # Last frame = end-effector
        ax.scatter(
            [ee_pos[0]], [ee_pos[1]], [ee_pos[2]],
            color='tab:red', s=50, marker='X'
        )
    plt.show()
