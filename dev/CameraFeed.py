#!/usr/bin/env python3
import cv2
import numpy as np
import apriltag
import FilePathFinder
import math

"""
This script is designed to capture video from a camera, process the images to detect AprilTags 
(fiducial markers), and use that information to infer the geometry of a chessboard.
"""

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
        """
        Returns a dictionary mapping each tag's ID to its center.
        """
        centerMap = {}
        for d in detections:
            centerMap[d.tag_id] = d.center.astype(int) # For each tag, extracts its center coordinates (converted to integers).
        return centerMap
    
    def drawCenterCircleForTags(self, frame, centers):
        for id in centers.keys(): # For every detected center
            print(centers[id])
            cv2.circle(frame,tuple(centers[id]),3,(0,0,255),2) # Draws a small red circle on the frame at that position.
            cv2.putText(frame,str(id),tuple(centers[id]),cv2.FONT_HERSHEY_PLAIN, 2,(0,0,255),2) # Puts the tag's ID next to the circle.
    

    def calculateEuclidianDist(self, point0, point1):
        """
        Computes the Euclidean distance between two 2D points.
        Uses basic distance formula to return an integer value.
        """
        return int(math.sqrt((int(point0[0])-int(point1[0]))*(int(point0[0])-int(point1[0])) 
                        + (int(point0[1])-int(point1[1]))*(int(point0[1])-int(point1[1]))))

    #returns real number. Do not convert the output of this function into an int. Only convert the final coordinate
    def calculateSlope(self, point0, point1):
        """
        Computes the slope (rise over run) between two points.
        Prints x-values for debugging.
        Returns 0 if the x difference is zero to avoid division by zero.
        """
        print(f"p0x = {point0[0]}, p1x = {point1[0]}")
        if point0[0] - point1[0] == 0:
            return 0
        return (point0[1] - point1[1])/(point0[0] - point1[0])

    #logic for setting the boundaries of both the outer chess board and each square within the board
    #inputs might be off
    def get_chessboard_boundaries(self,detections):
        """
        Assumes three specific AprilTags (with IDs 0, 1, and 2) represent the three corners of the chessboard:

        TL2: A corner from the top-left tag.
        BL1: A corner from the bottom-left tag.
        BR0: A corner from the bottom-right tag.

        Returns these three points as the boundaries used for further calculations.
        """
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
        return [TL2, BL1, BR0]
    
    def getAxesIntervalDots(self,TL,BL,BR,frame):
        """
        - Calculates dimensions
        - Creates a grid
        - Transforms the grid
        - Returns the transformed points which represent the grid (or axes interval dots) on the actual chessboard.
        """
        width = self.calculateEuclidianDist(BL,BR) # Distance between BL (bottom-left) and BR (bottom-right).
        height = self.calculateEuclidianDist(TL,BL) # Distance between TL (top-left) and BL.

        sh,sw = frame.shape[:2]
        
        x = None
        y = None
        
        # Uses np.linspace to create 9 evenly spaced points across the width and height.
        x = np.linspace(0,width,9)
        y = np.linspace(0,height,9)

        # Uses np.meshgrid to create a grid (representing the chessboard squares).
        mx,my = np.meshgrid(x,y)

        # Constructs an affine transformation matrix (M) to map the grid from a standard coordinate 
        # space to the actual board defined by the detected corners.
        # Applies the transformation to all grid points.
        points = np.vstack([mx.ravel(), my.ravel()]).T # contains the grid points generated from meshgrid that need transformation
        cspaceoriginal= np.float32([[0,0],[width-1,0],[0,height-1]]) # defines three points in the standard coordinate space: origin, bottom-right, and top-left
        cspaceNew = np.float32([BL,BR,TL]) # defines the corresponding actual points on the detected board (BL, BR, TL)
        M = cv2.getAffineTransform(cspaceoriginal,cspaceNew)
        ones = np.ones((points.shape[0], 1), dtype=points.dtype)
        #add col of ones for homogeneous eq solving
        points_hom = np.hstack([points, ones])
        pt = np.dot(points_hom, M.T)
        
        return pt

    def plotDotsOnAxes(self,frame,points):
        """
        Draws small red dots on the frame at each point in the grid.
        This visualization helps confirm that the chessboard's grid is correctly calculated.
        """
        for p in points:
            cv2.circle(frame,(int(p[0]),int(p[1])),3,(0,0,255),-1)

    def drawLine(self, frame, p0,p1):
        """
        Draws a green line between two given points on the frame.
        Useful for visualizing board boundaries or axes.
        """
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
                points = self.getAxesIntervalDots(corners[0],corners[1],corners[2],frame)
                self.plotDotsOnAxes(frame,points)
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
    