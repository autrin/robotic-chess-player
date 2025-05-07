import cv2
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from jh1.visual import (
    PIECE_TAG_IDS,
)
from jh1.visual._homography_solver import GRID_SIZE


def board_overlay_plot(img, solver, board_bins, certainty_grid):
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    def project(pt):
        pt = np.array(pt, dtype=np.float32)
        if solver.adjust:
            center = np.array([5, 5], dtype=np.float32)
            pt = (pt - center) * (9 / 8) + center
        vec = np.array([pt[0], pt[1], 1.0])
        img_pt_h = np.linalg.inv(solver.mat_homography) @ vec
        return tuple(img_pt_h[:2] / img_pt_h[2])

    for i in range(9):
        ax.plot(*zip(project((i + 1, 1)), project((i + 1, 9))), color="tab:blue", linewidth=1)
        ax.plot(*zip(project((1, i + 1)), project((9, i + 1))), color="tab:blue", linewidth=1)

    cmap = mpl.colormaps.get_cmap("RdYlGn")
    norm = mpl.colors.Normalize(vmin=0, vmax=1)

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            certainty = certainty_grid[i, j]
            if certainty <= 0:
                continue

            corners = [(j + dx, i + dy) for dx, dy in [(1, 1), (2, 1), (2, 2), (1, 2)]]
            img_corners = [project(c) for c in corners]
            poly = plt.Polygon(img_corners, color=cmap(norm(certainty)), alpha=0.7)
            ax.add_patch(poly)

            tags = board_bins[i][j]
            if tags:
                label = ", ".join(PIECE_TAG_IDS.get(t.tag_id, str(t.tag_id)) for t in tags)
                center = project((j + 1.5, i + 1.5))
                ax.text(*center, label, fontsize=10, ha="center", va="center", weight="bold",
                        color="black")

    sm = mpl.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    fig.colorbar(sm, ax=ax, fraction=0.026, pad=0.04).set_label("certainty", fontsize=9)
    ax.axis("off")
    plt.tight_layout()
    plt.show()
