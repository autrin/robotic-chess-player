import numpy as np
import cv2
from typing import List, Tuple, Optional

from ._tag_variational_cluster import TagVariationalCluster
from jh1.typealias import List2D

GRID_SIZE = 8


class HomographySolver:
    def __init__(self, corner_tags: List[Optional[TagVariationalCluster]], adjust: bool = True):
        """
        :param corner_tags: Must be 4 valid TagVariationalCluster objects representing the
                            centers of the corner squares (ordered CCW: bottom-left, bottom-right,
                            top-right, top-left).
        :param adjust: If True, scale down the board coordinates by a factor 8/9 pivoted at the center.
        """
        assert all(corner_tags) and len(corner_tags) == 4, "Need 4 valid corner tags"
        self.corner_tags = corner_tags
        self.adjust = adjust
        self.mat_homography = self._compute_homography()


    def _compute_homography(self) -> np.ndarray:
        image_pts = np.array([tag.position for tag in self.corner_tags], dtype=np.float32)
        board_pts = np.array([
            [0, 0],
            [10, 0],
            [10, 10],
            [0, 10]
        ], dtype=np.float32)
        mat_h, _ = cv2.findHomography(image_pts, board_pts)
        return mat_h


    def project_point(self, img_pt: Tuple[float, float]) -> Tuple[float, ...]:
        pt = np.array([img_pt[0], img_pt[1], 1.0])
        board_pt_h = self.mat_homography @ pt
        board_pt = board_pt_h[:2] / board_pt_h[2]
        if self.adjust:
            board_pt = HomographySolver._adjust_for_center(board_pt)
        return tuple(board_pt)


    def bin_pieces(self, pieces: List[TagVariationalCluster]) -> List2D[List[TagVariationalCluster]]:
        board_bins = [[[] for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        for tag in pieces:
            x, y = self.project_point(tag.position)
            if 1 <= x < 9 and 1 <= y < 9:
                i = int(y - 1)  # row
                j = int(x - 1)  # col
                board_bins[i][j].append(tag)
        return board_bins


    @staticmethod
    def get_certainty_grid(
        board_bins: List2D[List[TagVariationalCluster]],
        scale: float = 50.0
    ) -> np.ndarray:
        certainty = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                total = sum(t.detection_count for t in board_bins[i][j])
                certainty[i, j] = min(total / scale, 1.0)
        return certainty


    @staticmethod
    def _adjust_for_center(pt: np.ndarray) -> np.ndarray:
        # Scale the computed board coordinate about the board center ([5, 5]) with a factor of 8/9.
        center = np.array([5, 5])
        return (pt - center) * (8 / 9) + center
