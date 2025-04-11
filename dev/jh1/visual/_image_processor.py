from collections import Counter
from typing import Dict, Tuple, List

import numpy as np
import cv2
import pupil_apriltags as apriltag

from jh1.typealias import *

class ImageProcessor:
    """
    ImageProcessor provides AprilTag detection utilities with randomized augmentation and clustering.

    It includes preprocessing steps for contrast enhancement, affine augmentation with random perturbations,
    and robust AprilTag detection using the `pupil_apriltags` library. Clustered detections are aggregated
    and reduced via majority voting over tag IDs.

    Attributes:
        detector (apriltag.Detector): AprilTag detector instance.
        mode (str): Optional mode selector (e.g. "block", currently unused).
    """

    def __init__(self, mode="block"):
        """
        Initializes the ImageProcessor with a configured AprilTag detector.

        Args:
            mode (str): Optional mode flag (unused in current implementation).
        """
        self.detector = apriltag.Detector(
            families="tag25h9",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.4,
        )
        self.mode = mode

    def preprocess(self, frame: Array3D[uint8]) -> Array2D[uint8]:
        """
        Preprocesses the input frame by converting to grayscale, applying histogram equalization,
        CLAHE, and gamma correction.

        Args:
            frame (Array3D[uint8]): BGR input image.

        Returns:
            Array2D[uint8]: Preprocessed grayscale image.
        """
        gray: Array2D[uint8] = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray: Array2D[uint8] = cv2.equalizeHist(gray)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray: Array2D[float64] = clahe.apply(gray)
        return self.adjust_gamma(gray)

    def detect_tags(self, image: Array3D) -> apriltag.Detection:
        """
        Runs AprilTag detection on a given image.

        Args:
            image (Array3D): Input image (ideally grayscale).

        Returns:
            apriltag.Detection: DeItected AprilTags.
        """
        return self.detector.detect(image)

    def adjust_gamma(self, image: Array2D[uint8], gamma=1.5) -> Array2D[uint8]:
        """
        Applies gamma correction to an image.

        Args:
            image (Array2D[uint8]): Input grayscale image.
            gamma (float): Gamma value.

        Returns:
            Array2D[uint8]: Gamma-corrected image.
        """
        inv_gamma = 1.0 / gamma
        table: NDArray[uint8] = np.array([
            ((i / 255.0) ** inv_gamma) * 255 for i in range(256)
        ]).astype("uint8")
        return cv2.LUT(image, table)

    @staticmethod
    def find_april_tags(img: Array3D[uint8], detector: apriltag.Detector, n_random=50) -> Dict[int, List[Vec2]]:
        """
        Applies randomized affine transformations to detect AprilTags more robustly across augmented versions.

        Args:
            img (Array3D[uint8]): Input BGR image.
            detector (apriltag.Detector): Detector object to use.
            n_random (int): Number of random augmentations to apply.

        Returns:
            Dict[int, List[Vec2]]: Mapping from tag_id to list of (x, y) positions in original image space.
        """
        gray: Array2D[uint8] = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        detections: Dict[int, List[Vec2]] = {}

        cx, cy = w / 2, h / 2
        mat_t1 = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1]])
        mat_t2 = np.array([[1, 0, cx], [0, 1, cy], [0, 0, 1]])

        for _ in range(n_random):
            # Random transformation parameters
            d_theta = np.random.uniform(-np.pi, np.pi)
            d_scale = np.random.uniform(0.9, 1.1)
            d_shear = np.random.uniform(-0.4, 0.4)
            translate_x = np.random.uniform(-80, 80)
            translate_y = np.random.uniform(-80, 80)
            d_luma = np.random.uniform(-40, 40)
            d_contrast = np.random.uniform(0.5, 2.0)

            # Compose affine transformation matrix
            mat_rot = np.array([
                [np.cos(d_theta), -np.sin(d_theta), 0],
                [np.sin(d_theta), np.cos(d_theta), 0],
                [0, 0, 1]
            ])
            mat_scale = np.array([[d_scale, 0, 0], [0, d_scale, 0], [0, 0, 1]])
            mat_shear = np.array([[1, d_shear, 0], [0, 1, 0], [0, 0, 1]])
            mat_linear_trans = mat_rot @ mat_shear @ mat_scale
            mat_translate = np.array([[1, 0, translate_x], [0, 1, translate_y], [0, 0, 1]])
            mat_affine_trans = mat_t2 @ mat_translate @ mat_linear_trans @ mat_t1

            # Apply transformation and brightness/contrast adjustment
            warped = cv2.warpAffine(
                gray, mat_affine_trans[:2, :], (w, h),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE
            )
            warped = cv2.convertScaleAbs(warped, alpha=d_contrast, beta=d_luma)

            # Detect tags in transformed image
            tags_affine = detector.detect(warped)
            mat_affine_inv = np.linalg.inv(mat_affine_trans)

            # Map detections back to original image coordinates
            # noinspection PyTypeChecker
            for td in tags_affine:
                center_affine = np.array([td.center[0], td.center[1], 1])
                center = mat_affine_inv @ center_affine
                detections.setdefault(td.tag_id, []).append((center[0], center[1]))

        return detections

    @staticmethod
    def count_clusters(detections: Dict[int, List[Vec2]], radius: float = 16) -> List[Tuple[int, int, Vec2]]:
        """
        Groups tag detections into spatial clusters, then selects the majority tag ID per cluster.

        Args:
            detections (Dict[int, List[Vec2]]): Mapping from tag_id to detected (x, y) positions.
            radius (float): Distance threshold for clustering nearby detections.

        Returns:
            List[Tuple[int, int, Vec2]]: List of clusters. Each cluster is a tuple:
                - majority tag_id (int)
                - count of points in cluster (int)
                - center (x, y) of cluster (Vec2)
        """
        all_points = [
            (tag_id, np.array(pos))
            for tag_id, positions in detections.items()
            for pos in positions
        ]
        used = np.zeros(len(all_points), dtype=bool)
        clusters = []

        for i, (tag_id, pt) in enumerate(all_points):
            if used[i]: continue

            cluster_ids = [tag_id]
            cluster_pts = [pt]
            used[i] = True

            for j in range(i + 1, len(all_points)):
                if used[j]: continue
                candidate_id, candidate_xy = all_points[j]
                if np.linalg.norm(pt - candidate_xy) <= radius:
                    cluster_ids.append(candidate_id)
                    cluster_pts.append(candidate_xy)
                    used[j] = True

            if len(cluster_pts) > 1:
                majority_id = Counter(cluster_ids).most_common(1)[0][0]
                cluster_center = np.mean(cluster_pts, axis=0)
                clusters.append((
                    majority_id,
                    len(cluster_pts),
                    (float(cluster_center[0]), float(cluster_center[1]))
                ))

        return clusters
