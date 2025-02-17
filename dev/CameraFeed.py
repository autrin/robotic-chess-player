#!/usr/bin/env python3
import cv2
import numpy as np
import sys

class CameraFeed:
    #use april tags
    def __init__(self,camID = 0):
        
        self.camID = camID
        self.meanbgr = []
        #bgr ordering
        

    def openCampera(self):
        self.cam = cv2.VideoCapture(self.camID)
        if not self.cam.isOpened():
            print(f"ERROR: can't open camera id={self.camID}")
            exit()
    
   

    def destroyCameraFeed(self, destroyAllWindows=True):
        self.cam.release()
        if destroyAllWindows:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    ocr = CameraFeedOCR(0)
    ocr.openCampera()
    
    while True:
        ret,frame = ocr.cam.read()
        cv2.imshow(f"FEED Cam-ID = {ocr.camID}",frame)
    
        print(ocr.meanbgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    ocr.destroyCameraFeed()
    