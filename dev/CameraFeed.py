#!/usr/bin/env python3
import cv2
import numpy as np
import apriltag
import FilePathFinder

class CameraFeed:
    #use april tags
    def __init__(self,camID = 0):
        self.cam = None
        self.camID = camID
        self.aprilDetector = apriltag.Detector()

        

    def openCampera(self):
        self.cam = cv2.VideoCapture(self.camID)
        if not self.cam.isOpened():
            print(f"ERROR: can't open camera id={self.camID}")
            exit()
    
    #this might be unnecessary
    def getCenterPositionofDetection(self,detections):
        centerMap = {}
        for d in detections:
            centerMap[d.tag_id] = d.center.astype(int)
        return centerMap
    
    def drawCenterCircleForTags(self, frame, centers):
        for id in centers.keys():
            print(centers[id])
            cv2.circle(frame,tuple(centers[id]),3,(0,0,255),2)
            cv2.putText(frame,str(id),tuple(centers[id]),cv2.FONT_HERSHEY_PLAIN, 2,(0,0,255),2)
        
    def adjust_gamma(self,image, gamma=1.5):
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                        for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)

    def startLoop(self):
        while True:
            ret,frame = self.cam.read()
            
            grayFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            
            grayFrame = cv2.equalizeHist(grayFrame) #Boost contrast
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            grayFrame = clahe.apply(grayFrame)
            grayFrame = self.adjust_gamma(grayFrame)
            
            detections = self.aprilDetector.detect(img=grayFrame)
            centers = self.getCenterPositionofDetection(detections)
            self.drawCenterCircleForTags(frame,centers)
            cv2.imshow(f"FEED Cam-ID = {self.camID}",frame)
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
    