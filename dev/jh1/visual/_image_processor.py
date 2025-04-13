from collections import Counter
from typing import Dict, Tuple, List

import numpy as np
import cv2
import pupil_apriltags as apriltag

from jh1.typealias import *
from ._tag_variational_cluster import TagVariationalCluster


def preprocess(frame: Array3D[uint8]) -> Array2D[uint8]:
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
    return adjust_gamma(gray)


def adjust_gamma(image: Array2D[uint8], gamma=1.5) -> Array2D[uint8]:
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


def _generate_image_perturbation() -> Tuple[float, ...]:
    # Generate small random perturbations:
    d_theta = np.random.uniform(-np.pi, np.pi)
    # scaling factor
    d_scale = np.random.uniform(0.75, 1.25)
    # shear factor (skew)
    d_shear = np.random.uniform(-0.4, 0.4)
    # translation in x (pixels)
    tx = np.random.uniform(-300, 300)
    # translation in y (pixels)
    ty = np.random.uniform(-80, 80)
    # brightness adjustment
    d_luma = np.random.uniform(-50, 50)
    # contrast adjustment
    d_contrast = np.random.uniform(0.4, 2.6)
    return d_theta, d_scale, d_shear, tx, ty, d_luma, d_contrast


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
    gray: Array2D[uint8] = preprocess(img)
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
        d_theta, d_scale, d_shear, tx, ty, d_luma, d_contrast = _generate_image_perturbation()

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


def count_clusters(detections: Dict[int, List[Vec2]], radius: float = 12) -> List[TagVariationalCluster]:
    """
    Groups tag detections into spatial clusters, then selects the majority tag ID per cluster.

    Args:
        detections (Dict[int, List[Vec2]]): Mapping from tag_id to detected (x, y) positions.
        radius (float): Distance threshold for clustering nearby detections.

    Returns:
        List[TagVariationalCluster]: List of TagVariationalCluster objects. Each cluster includes:
            - majority tag_id (int)
            - count of points in cluster (int)
            - center (x, y) of cluster (Vec2)
            - pooled standard deviation (float)
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
    clusters: List[TagVariationalCluster] = []

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
        if len(cluster_pts) <= 1: continue

        # Choose majority ID among clustered tag IDs
        majority_id = Counter(cluster_ids).most_common(1)[0][0]

        # Compute mean center of the cluster
        cluster_pts_arr = np.stack(cluster_pts, axis=0)
        cluster_center = np.mean(cluster_pts_arr, axis=0)

        # Compute pooled std dev: sqrt(1/n * sum(||x_i - mean||^2))
        diffs = cluster_pts_arr - cluster_center
        sq_norms = np.sum(diffs ** 2, axis=1)
        pooled_stdev = float(np.sqrt(np.mean(sq_norms)))

        clusters.append(TagVariationalCluster(
            tag_id=majority_id,
            detection_count=len(cluster_pts),
            position=(float(cluster_center[0]), float(cluster_center[1])),
            stdev=pooled_stdev
        ))

    return clusters
