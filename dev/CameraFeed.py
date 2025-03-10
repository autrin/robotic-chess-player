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
        self.referenceImage = cv2.imread("../resources/ChessboardReference.png")
        #self.referenceImage = self.convertToTagDetectableImage(self.referenceImage)

    def convertToTagDetectableImage(self, image):
        grayFrame = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        grayFrame = cv2.equalizeHist(grayFrame) #Boost contrast
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        grayFrame = clahe.apply(grayFrame)
        grayFrame = self.adjust_gamma(grayFrame)
        return grayFrame


        
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
        print(f"p0x = {point0[0]}, p1x = {point1[0]}")
        if point0[0] - point1[0] == 0:
            return 0
        return (point0[1] - point1[1])/(point0[0] - point1[0])

    #logic for setting the boundaries of both the outer chess board and each square within the board
    #inputs might be off
    #returns a pair of 2 lists- one containing the left edges and one containing the top edges
    def get_chessboard_boundaries(self,detections):
        if detections == None or len(detections) != 4:
            return None
        #TL = top left apriltag, BR = bottom right apriltag, BL = bottom left apriltag ...
        
        TL = detections[0].corners
        BL= detections[1].corners        
        BR = detections[2].corners
        TR = detections[3].corners
        
        TL = [int(TL[2][0]),int(TL[2][1])]
        BL = [int(BL[1][0]),int(BL[1][1])]
        BR = [int(BR[0][0]),int(BR[0][1])]
        TR = [int(TR[3][0]), int(TR[3][1])]
        return [TL, BL, BR, TR]
    
    def getHomoGraphicAppliedImage(self,sourceImage,fourPoints, refImg = None, ow = 900, oh = 800):
        if not refImg:
            refImg = self.referenceImage
        
        refdi = cv2.cvtColor(refImg,cv2.COLOR_BGR2GRAY)
        refImgDetections = self.aprilDetector.detect(img=refdi)
        #cv2.imshow(f"warped",refdi)
        #return None
        if not refImgDetections:
            return None
        
        refImgPoints = self.get_chessboard_boundaries(refImgDetections)
        
        if not refImgPoints:
            return None
        
        cv2.imshow(f"warped",refdi)
        source = np.float32(fourPoints)
        ref = np.float32(refImgPoints)
        print(f"source == {source}")
        mat = cv2.getPerspectiveTransform(source,ref)
        return cv2.warpPerspective(sourceImage,mat,(ow,oh))
    
    def getAxesIntervalDots(self,TL,BL,BR):
        width = self.calculateEuclidianDist(BL,BR)
        height = self.calculateEuclidianDist(TL,BL)

        x = np.linspace(0,width,9)
        y = np.linspace(0,height,9)
        mx,my = np.meshgrid(x,y)
        
    
        return mx, my

    
    def plotDotsOnAxes(self,frame,mx,my,TL):
        for i in range(my.shape[0]):
            for j in range(mx.shape[1]):
                x = int(mx[i, j]) + TL[0]
                y = int(my[i, j]) + TL[1]
                cv2.circle(frame, (x, y), 3,(0, 0, 255), -1)








    def drawLine(self, frame, p0,p1):
        cv2.line(frame, p0, p1, color=(0, 255, 0), thickness=1)


    #Get the april tags responsible for the board corners
    def getChessBoardCorners(self,detections):
        ret = []
        
        for d in detections:
            if (d.tag_id == 0 or d.tag_id ==1 or d.tag_id ==2 or d.tag_id == 3) and d.tag_id not in ret:
                ret.append(d)
         
        if len(ret) != 4:
            print("Failed to detect all four corners, detecting april tags again")
            return None
        #sort the array
        ret = sorted(ret,key=lambda x : x.tag_id)        
        return ret
    
    


    def adjust_gamma(self,image, gamma=1.5):
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                        for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)
    
    #for debugging
    def drawAprilTagCorner(self,frame,detections,cornerPos):
        for d in detections:
            cv2.circle(frame,(int(d.corners[cornerPos][0]),int(d.corners[cornerPos][1])),3,(255,0,255),2)

    def drawBordersandDots(self,frame,detections,grayFrame=None):
            if grayFrame is None:
                grayFrame = frame

            centers = self.getCenterPositionofDetection(detections)
            self.drawCenterCircleForTags(frame,centers)
            cbc = self.getChessBoardCorners(detections)
            if cbc:
                corners = self.get_chessboard_boundaries(cbc)
                mx,my = self.getAxesIntervalDots(corners[0],corners[1],corners[2])
                self.plotDotsOnAxes(frame,mx,my,corners[0])
                self.drawLine(frame,corners[0],corners[1])
                self.drawLine(frame,corners[1],corners[2])
                print("DRAWING")
            
    def startLoop(self):
        while True:
            ret,frame = self.cam.read()
            
            grayFrame = self.convertToTagDetectableImage(frame)
            detections = self.aprilDetector.detect(img=grayFrame)
            #self.drawBordersandDots(frame,detections,grayFrame)
            
            if detections:
                fp = self.get_chessboard_boundaries(detections)
                if fp:
                    warpedFrame = self.getHomoGraphicAppliedImage(grayFrame,fp)
                    if warpedFrame is not None:
                        detections = self.aprilDetector.detect(img=warpedFrame)
                        warpedFrame = cv2.cvtColor(warpedFrame,cv2.COLOR_GRAY2BGR)
                        self.drawBordersandDots(warpedFrame,detections)
                        cv2.imshow(f"warped",warpedFrame)


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
    