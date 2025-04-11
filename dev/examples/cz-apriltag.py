import cv2
import numpy as np
import pupil_apriltags as apriltag
import time

cap = cv2.VideoCapture(0)
detector = apriltag.Detector(
    families="tag25h9",
    nthreads=12,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.2,
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 900)

n_perturbs = 60


def find_april_tags(img, detector, n_random=n_perturbs):
    # Convert to grayscale and get image dimensions
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray = img
    h, w = gray.shape

    # Dictionary to store detections with tag id as key and list of positions as value
    detections = {}

    # Pre-calculate image center for affine transforms
    cx, cy = w / 2, h / 2
    T1 = np.array([[1, 0, -cx],
                   [0, 1, -cy],
                   [0, 0, 1]])
    T2 = np.array([[1, 0, cx],
                   [0, 1, cy],
                   [0, 0, 1]])

    for _ in range(n_random):
        # Generate small random perturbations:
        angle = np.random.uniform(-180, 180)  # degrees
        scale = np.random.uniform(0.8, 1.2)
        shear = np.random.uniform(-0.4, 0.4)  # shear factor (skew)
        tx = np.random.uniform(-80, 80)  # translation in x (pixels)
        ty = np.random.uniform(-80, 80)  # translation in y (pixels)
        brightness = np.random.uniform(-60, 60)  # brightness adjustment
        contrast = np.random.uniform(0.3, 3.0)  # contrast adjustment

        # Build the transformation matrices (in homogeneous coordinates)
        theta = np.deg2rad(angle)
        R = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta), np.cos(theta), 0],
                      [0, 0, 1]])
        S = np.array([[scale, 0, 0],
                      [0, scale, 0],
                      [0, 0, 1]])
        Sh = np.array([[1, shear, 0],
                       [0, 1, 0],
                       [0, 0, 1]])

        # Compose transformation: rotation, shear, then scaling.
        A = R @ Sh @ S

        # Build translation matrix (in homogeneous coordinates)
        T_translate = np.array([[1, 0, tx],
                                [0, 1, ty],
                                [0, 0, 1]])

        # Total transformation: center shift, then transform, then shift back
        M_total = T2 @ T_translate @ A @ T1
        M_affine = M_total[:2, :]  # 2x3 matrix for cv2.warpAffine

        # Apply the geometric transformation
        warped = cv2.warpAffine(gray, M_affine, (w, h),
                                flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_REPLICATE)
        # Apply brightness and contrast adjustments
        warped = cv2.convertScaleAbs(warped, alpha=contrast, beta=brightness)

        # Run the apriltag detector on the perturbed image
        detections_transformed = detector.detect(warped)

        # Compute the inverse affine transformation matrix
        M_inv = np.linalg.inv(M_total)
        for det in detections_transformed:
            # Get the detection center in warped coordinates and convert to homogeneous coordinate
            center_warped = np.array([det.center[0], det.center[1], 1])
            # Inverse transform to get original image coordinates
            original_center = M_inv @ center_warped
            tag_id = det.tag_id
            # Add the position to the list for this tag id
            detections.setdefault(tag_id, []).append((original_center[0], original_center[1]))

    # Return the dictionary mapping tag id to a list of positions
    return detections


TARGET_FPS = 10
FRAME_DELAY = 1.0 / TARGET_FPS
last_time = time.time()

results = {}



def count_clusters(detections, radius=16):
    """
    Takes in detection results from find_april_tags and returns a list of clusters.

    Each cluster is a tuple: ([id1, id2, ...], count, (x, y))
    """
    # Flatten detections into a list of (tag_id, (x, y))
    all_points = [(tag_id, np.array(pos)) for tag_id, positions in detections.items() for pos in positions]
    used = np.zeros(len(all_points), dtype=bool)
    clusters = []

    for i, (tag_id, pt) in enumerate(all_points):
        if used[i]:
            continue

        # Start new cluster with this point
        cluster_ids = [tag_id]
        cluster_pts = [pt]
        used[i] = True

        # Find neighbors within radius
        for j in range(i + 1, len(all_points)):
            if used[j]:
                continue
            _, candidate_pt = all_points[j]
            if np.linalg.norm(pt - candidate_pt) <= radius:
                cluster_ids.append(all_points[j][0])
                cluster_pts.append(candidate_pt)
                used[j] = True

        cluster_ids = list(set(cluster_ids))  # unique ids
        cluster_center = np.mean(cluster_pts, axis=0)
        clusters.append((cluster_ids, len(cluster_pts), (float(cluster_center[0]), float(cluster_center[1]))))

    return clusters



while True:
    ret, frame = cap.read()
    if not ret:
        break

    print(np.shape(frame))

    current_time = time.time()
    results = []
    cluster_counts = []
    if current_time - last_time >= FRAME_DELAY:
        last_time = current_time
        results = find_april_tags(frame, detector)
        cluster_counts = count_clusters(results)


    # Draw a dot for every detected position regardless of id
    for tag_id, positions in results.items():
        for pos in positions:
            center = (int(pos[0]), int(pos[1]))
            cv2.circle(frame, center, radius=3, color=(0, 0, 255), thickness=-1)

    # Draw cluster metadata (id array and count)
    for cluster_ids, count, center in cluster_counts:
        text = f"{count / n_perturbs:.0%}"
        center_int = (int(center[0]), int(center[1]))
        cv2.putText(frame, text, center_int, cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (10 + (count / n_perturbs) * 100, 10, 100 + (count / n_perturbs) * 150), 2, cv2.LINE_AA)

    cv2.imshow("AprilTags", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
