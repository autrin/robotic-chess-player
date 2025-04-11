import cv2
import numpy as np
import pupil_apriltags as apriltag
import time

from jh1.visual import ImageProcessor

cap = cv2.VideoCapture(0)
detector = apriltag.Detector(
    families="tag25h9",
    nthreads=12,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.2,
)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

n_perturbs = 60

TARGET_FPS = 10
FRAME_DELAY = 1.0 / TARGET_FPS
last_time = time.time()

results = {}





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
        results = ImageProcessor.find_april_tags(frame, detector)
        cluster_counts = ImageProcessor.count_clusters(results)


    # Draw a dot for every detected position regardless of id
    for tag_id, positions in results.items():
        for pos in positions:
            center = (int(pos[0]), int(pos[1]))
            cv2.circle(frame, center, radius=3, color=(0, 0, 255), thickness=-1)

    # Draw cluster metadata (id array and count)
    for cluster_ids, count, center in cluster_counts:
        text = f"{cluster_ids} {count / n_perturbs:.0%}"
        center_int = (int(center[0]), int(center[1]))
        cv2.putText(frame, text, center_int, cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (10, 10, 100 + (count / n_perturbs) * 150), 1, cv2.LINE_AA)

    cv2.imshow("AprilTags", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
