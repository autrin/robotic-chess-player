import cv2
import mss
import numpy as np

from ._abstract_video_source import AbstractVideoSource


class ScreenSource(AbstractVideoSource):
    def __init__(self, region=None):
        self.sct = mss.mss()
        self.region = region or {"top": 0, "left": 0, "width": 1250, "height": 900}

    def read_frame(self):
        screenshot = self.sct.grab(self.region)
        frame = np.array(screenshot)
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def release(self):
        self.sct.close()
