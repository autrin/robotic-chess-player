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
    def _generate_image_perturbation() -> Tuple[float, ...]:
        # Generate small random perturbations:
        d_theta = np.random.uniform(-np.pi, np.pi)
        # scaling factor
        d_scale = np.random.uniform(0.9, 1.1)
        # shear factor (skew)
        d_shear = np.random.uniform(-0.4, 0.4)
        # translation in x (pixels)
        tx = np.random.uniform(-80, 80)
        # translation in y (pixels)
        ty = np.random.uniform(-80, 80)
        # brightness adjustment
        d_luma = np.random.uniform(-40, 40)
        # contrast adjustment
        d_contrast = np.random.uniform(0.5, 2.0)
        return d_theta, d_scale, d_shear, tx, ty, d_luma, d_contrast


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
        # Convert to grayscale and get image dimensions
        gray: Array2D[uint8] = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # Dictionary to store detections with tag id as key and list of positions as value
        detections: Dict[int, List[Vec2]] = {}

        # Pre-calculate image center for affine transforms
        cx, cy = w / 2, h / 2
        mat_t1: Mat3x3 = np.array([[1, 0, -cx],
                                   [0, 1, -cy],
                                   [0, 0, 1]])
        mat_t2: Mat3x3 = np.array([[1, 0, cx],
                                   [0, 1, cy],
                                   [0, 0, 1]])

        for _ in range(n_random):
            d_theta, d_scale, d_shear, tx, ty, d_luma, d_contrast = ImageProcessor._generate_image_perturbation()

            # Build the linear component transformation matrices (in homogeneous coordinates)
            mat_rot: Mat3x3 = np.array([
                [np.cos(d_theta), -np.sin(d_theta), 0],
                [np.sin(d_theta), np.cos(d_theta), 0],
                [0, 0, 1]
            ])
            mat_scale: Mat3x3 = np.array([
                [d_scale, 0, 0],
                [0, d_scale, 0],
                [0, 0, 1]
            ])
            mat_shear: Mat3x3 = np.array([
                [1, d_shear, 0],
                [0, 1, 0],
                [0, 0, 1]
            ])

            # Compose transformation: rotation, shear, then scaling.
            mat_linear_trans: Mat3x3 = mat_rot @ mat_shear @ mat_scale

            # Build translation matrix (in homogeneous coordinates)
            mat_translate: Mat3x3 = np.array([
                [1, 0, tx],
                [0, 1, ty],
                [0, 0, 1]
            ])

            # Total transformation: center shift, then transform, then shift back
            mat_affine_trans: Mat3x3 = mat_t2 @ mat_translate @ mat_linear_trans @ mat_t1

            # Apply the geometric transformation
            gray_affine_sp: Array2D[uint8] = cv2.warpAffine(
                src=gray,
                M=mat_affine_trans[:2, :],  # 2x3 matrix for cv2.warpAffine
                dsize=(w, h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REPLICATE
            )
            # Apply brightness and contrast adjustments
            gray_affine_sp: Array2D[uint8] = cv2.convertScaleAbs(gray_affine_sp, alpha=d_contrast, beta=d_luma)

            # Run the apriltag detector on the perturbed image
            tags_affine_sp: apriltag.Detection = detector.detect(gray_affine_sp)

            # Compute the inverse affine transformation matrix
            mat_affine_inv: Mat3x3 = np.linalg.inv(mat_affine_trans)
            # noinspection PyTypeChecker
            for td in tags_affine_sp:
                # Get the detection center in warped coordinates and convert to homogeneous coordinate
                center_affine_sp: Vec3 = np.array([td.center[0], td.center[1], 1])

                # Inverse transform to get original image coordinates
                center: Vec3 = mat_affine_inv @ center_affine_sp

                # Add the position to the list for this tag id
                detections.setdefault(td.tag_id, []).append((center[0], center[1]))

        # Return the dictionary mapping tag id to a list of positions
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
        # Flatten detections into a list of (tag_id, (x, y)) points
        all_points: List[Tuple[int, Vec2]] = [
            (tag_id, np.array(pos))
            for tag_id, positions in detections.items()
            for pos in positions
        ]

        # Track which points have been assigned to a cluster
        used: NDArray[bool] = np.zeros(len(all_points), dtype=bool)

        # Output list of clusters
        clusters: List[Tuple[int, int, Vec2]] = []

        # Iterate through all points to form clusters
        for i, (tag_id, pt) in enumerate(all_points):
            if used[i]: continue  # Skip already clustered points

            # Start new cluster
            cluster_ids: List[int] = [tag_id]
            cluster_pts: List[Vec2] = [pt]
            used[i] = True

            # Search for nearby unclustered points within the radius
            for j in range(i + 1, len(all_points)):
                if used[j]: continue

                candidate_id, candidate_xy = all_points[j]
                if np.linalg.norm(pt - candidate_xy) <= radius:
                    cluster_ids.append(candidate_id)
                    cluster_pts.append(candidate_xy)
                    used[j] = True

            # Only form cluster if more than one point supports it
            if len(cluster_pts) > 1:
                # Choose majority ID among clustered tag IDs
                majority_id = Counter(cluster_ids).most_common(1)[0][0]
                # Compute mean center of the cluster
                cluster_center: Vec2 = np.mean(cluster_pts, axis=0)
                clusters.append((
                    majority_id,
                    len(cluster_pts),
                    (float(cluster_center[0]), float(cluster_center[1]))
                ))

        return clusters
