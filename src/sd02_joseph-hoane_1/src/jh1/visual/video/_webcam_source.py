import cv2

from ._abstract_video_source import AbstractVideoSource
from jh1.typealias import *


class WebcamSource(AbstractVideoSource):
    def __init__(self, cam_id=0, res_x=1280, res_y=720):
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)

    def read_frame(self) -> Array3D[uint8]:
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        self.cap.release()
