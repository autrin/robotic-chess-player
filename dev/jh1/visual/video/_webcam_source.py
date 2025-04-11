import cv2

from ._abstract_video_source import AbstractVideoSource


class WebcamSource(AbstractVideoSource):
    def __init__(self, cam_id=0):
        self.cap = cv2.VideoCapture(cam_id)

    def read_frame(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        self.cap.release()