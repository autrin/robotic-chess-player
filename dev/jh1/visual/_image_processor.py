import cv2
import numpy as np
import pupil_apriltags as apriltag


class ImageProcessor:
    def __init__(self, mode="block"):
        self.detector = apriltag.Detector(
            families="tag25h9"
        )
        self.mode = mode

    def preprocess(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        return self.adjust_gamma(gray)

    def detect_tags(self, image):
        return self.detector.detect(image)

    def adjust_gamma(self, image, gamma=1.5):
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)
