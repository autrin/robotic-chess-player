#!/usr/bin/env python3
import cv2
import numpy as np
import apriltag

class CameraFeed:
    #use april tags
    def __init__(self,camID = 0):
        self.cam = None
        self.camID = camID
        self.aprilDetector = apriltag.Detector

        

    def openCampera(self):
        self.cam = cv2.VideoCapture(self.camID)
        if not self.cam.isOpened():
            print(f"ERROR: can't open camera id={self.camID}")
            exit()
    
    def startLoop(self):
        while True:
            ret,frame = self.cam.read()
            cv2.imshow(f"FEED Cam-ID = {self.camID}",frame)
        
            #print(ocr.meanbgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

   

    def destroyCameraFeed(self, destroyAllWindows=True):
        self.cam.release()
        if destroyAllWindows:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    ocr = CameraFeed(0)
    ocr.openCampera()
    ocr.startLoop()
    
    ocr.destroyCameraFeed()
    