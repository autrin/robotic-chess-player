import numpy as np
import cv2
from typing import List, Tuple, Optional

from ._vari_cluster import VariCluster
from jh1.typealias import *

GRID_SIZE = 8


class HomographySolver:
    """
    Computes a homography transformation from image space to a canonical board coordinate space.
    Used to project AprilTag detections onto an 8x8 chessboard grid.

    The board coordinate system is defined as a 10x10 square with corner squares centered at
    (0, 0), (10, 0), (10, 10), and (0, 10), in counter-clockwise order starting from bottom-left.
    """

    def __init__(self, corner_tags: List[Optional[VariCluster]], adjust: bool = True):
        """
        Initialize the HomographySolver.

        :param corner_tags: A list of 4 TagVariationalCluster instances representing the centers
                            of the four corner squares of the chessboard. They must be ordered
                            counter-clockwise starting from bottom-left.
        :param adjust: If True, projected board coordinates are scaled about the center to align
                       an 8x8 grid within the 10x10 canonical square.
        """
        assert all(corner_tags) and len(corner_tags) == 4, "Need 4 valid corner tags"
        self.corner_tags: List[VariCluster] = corner_tags
        self.adjust: bool = adjust
        self.mat_homography: Mat3x3 = self._compute_homography()

    def _compute_homography(self) -> Mat3x3:
        """
        Compute the homography matrix that maps image points to board coordinates.

        :return: 3x3 homography matrix as a NumPy array.
        """
        image_pts: Array2D[np.float32] = np.array([tag.position for tag in self.corner_tags],
                                                  dtype=np.float32)
        board_pts: Array2D[np.float32] = np.array([
            [0, 0],
            [10, 0],
            [10, 10],
            [0, 10]
        ], dtype=np.float32)
        mat_h, _ = cv2.findHomography(image_pts, board_pts)
        return mat_h

    def project_point(self, img_pt: Vec2) -> Tuple[float, ...]:
        """
        Project a point from image space to canonical board coordinates.

        :param img_pt: (x, y) tuple in image coordinates.
        :return: (x, y) tuple in board coordinates (adjusted if enabled).
        """
        pt: Vec3 = np.array([img_pt[0], img_pt[1], 1.0])
        board_pt_h: Vec3 = self.mat_homography @ pt
        board_pt: Vec2 = board_pt_h[:2] / board_pt_h[2]
        if self.adjust:
            board_pt = HomographySolver._adjust_for_center(board_pt)
        return tuple(board_pt)

    def bin_pieces(self, pieces: List[VariCluster]) -> List2D[List[VariCluster]]:
        """
        Assign projected piece detections to board bins based on their location.

        :param pieces: List of TagVariationalCluster instances representing detected pieces.
        :return: 8x8 list-of-lists grid where each cell contains a list of clusters that fall into it.
        """
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
            board_bins: List2D[List[VariCluster]],
            scale: float = 50.0
    ) -> Array2D[np.float64]:
        """
        Generate a heatmap-like certainty grid based on detection counts.

        :param board_bins: Output from `bin_pieces`.
        :param scale: Scaling factor to normalize detection counts into [0, 1].
        :return: 8x8 NumPy array of certainty values in [0, 1].
        """
        certainty = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                total = sum(t.detection_count for t in board_bins[i][j])
                certainty[i, j] = min(total / scale, 1.0)
        return certainty

    @staticmethod
    def _adjust_for_center(pt: Vec2) -> Vec2:
        """
        Apply a center-based scaling transform to map a 10x10 board space into 8x8.

        :param pt: Point in canonical 10x10 board coordinates.
        :return: Adjusted point in scaled 8x8 space centered around (5, 5).
        """
        center = np.array([5, 5])
        return (pt - center) * (8 / 9) + center
