#!/usr/bin/env python3
import cv2
import numpy as np
import apriltag
import FilePathFinder
import math

class CameraFeed:
    #use april tags
    def __init__(self,camID = 0):
        self.cam = None
        self.camID = camID
        self.aprilDetector = apriltag.Detector()

        
    def openCamera(self):
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
    

    def calculateEuclidianDist(self, point0, point1):
        return int(math.sqrt((int(point0[0])-int(point1[0]))*(int(point0[0])-int(point1[0])) 
                        + (int(point0[1])-int(point1[1]))*(int(point0[1])-int(point1[1]))))

    #returns real number. Do not convert the output of this function into an int. Only convert the final coordinate
    def calculateSlope(self, point0, point1):
        return (point0[1] - point1[1])/(point0[0] - point1[0])

    #logic for setting the boundaries of both the outer chess board and each square within the board
    #inputs might be off
    #returns a pair of 2 lists- one containing the left edges and one containing the top edges
    def get_chessboard_boundaries(self,detections):
        if detections == None or len(detections) != 3:
            return None
        #within the detections[] array, 1,2, and 3 are all just arbitrary. As such, 
        #we will need to fill the indices in with the 3 unique tags that correspond to the edges of the board
        #TL = top left apriltag, BR = bottom right apriltag, BL = bottom left apriltag
        print(detections)
        TL0,TL1,TL2,TL3 = detections[0].corners
        BL0,BL1,BL2,BL3 = detections[1].corners        
        BR0,BR1,BR2,BR3 = detections[2].corners

        # widthInterval = width/8
        # heightInterval = height/8
        
        # hSlope = self.calculateSlope(BL1,BR0)
        # vSlope = self.calculateSlope(TL2,BL1)
        # hMagInverse = (math.sqrt(1+hSlope**2))
        # vMagInverse = (math.sqrt(1+vSlope**2))
        # horizontalUnitVect = [1/hMagInverse, hSlope/hMagInverse]
        # verticalUnitVect = [1/vMagInverse,vSlope/vMagInverse]

        #refractoring, sorry cal
        TL2 = [int(TL2[0]),int(TL2[1])]
        BL1 = [int(BL1[0]),int(BL1[1])]
        BR0 = [int(BR0[0]),int(BR0[1])]
        return TL2, BL1, BR0

    def drawLine(self, frame, p0,p1):
        cv2.line(frame, p0, p1, color=(0, 255, 0), thickness=1)


    #Get the april tags responsible for the board corners
    def getChessBoardCorners(self,detections):
        ret = []
        
        for d in detections:
            if (d.tag_id == 0 or d.tag_id ==1 or d.tag_id ==2) and d.tag_id not in ret:
                ret.append(d)
        
        #If all the corners are not in the list, detect again 
        if len(ret) != 3:
            print("Failed to detect all three corners, detecting april tags again")
            return None
        #sort the array
        sorted(ret,key=lambda x : x.tag_id)        
        return ret
    
    


    def adjust_gamma(self,image, gamma=1.5):
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                        for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)
    
    #for debugging
    def drawAprilTagCorner(self,frame,detections,cornerPos):
        for d in detections:
            #print(centers[id])
            #print(type(d.corners[cornerPos][0]))
            cv2.circle(frame,(int(d.corners[cornerPos][0]),int(d.corners[cornerPos][1])),3,(255,0,255),2)

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

            cbc = self.getChessBoardCorners(detections)
            
            if cbc:
                corners = self.get_chessboard_boundaries(cbc)
                self.drawLine(frame,corners[0],corners[1])
                self.drawLine(frame,corners[1],corners[2])
                #self.drawAprilTagCorner(frame,detections,1)
            cv2.imshow(f"FEED Cam-ID = {self.camID}",frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


   

    def destroyCameraFeed(self, destroyAllWindows=True):
        self.cam.release()
        if destroyAllWindows:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    ocr = CameraFeed(1)
    ocr.openCamera()
    ocr.startLoop()
    
    ocr.destroyCameraFeed()
    