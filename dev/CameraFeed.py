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

class CameraFeedClass:
    #use april tags, mode := paper | block
    def __init__(self,camID = 0,mode="paper"):
        self.cam = None
        self.camID = camID
        self.aprilDetector = apriltag.Detector(apriltag.DetectorOptions(families="tag25h9"))
        #self.whiteChessPieceDetector = apriltag.Detector(apriltag.DetectorOptions(families="tag25h9"))
        #self.blackChessPieceDetector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.PieceDetector = apriltag.Detector(apriltag.DetectorOptions(families="tag25h9")) 
        self.mode = mode
        
        self.referenceImage = cv2.imread("../resources/ChessboardReference.png")
        self.hasBeenCalibrated = False
        self.chessBoardVertices = [] 
        self.chessBoardCells = []
        self.chessBoardResetCounter = 150
        self.chessBoardResetCounterThreshold = 150  
        self.previosBoard = [
            list("rnbqkbnr"),
            list("pppppppp"),
            list("........"),
            list("........"),
            list("........"),
            list("........"),
            list("PPPPPPPP"),
            list("RNBQKBNR")
        ] 

        self.currentBoard = [
            list("rnbqkbnr"),
            list("pppppppp"),
            list("........"),
            list("........"),
            list("........"),
            list("........"),
            list("PPPPPPPP"),
            list("RNBQKBNR")
        ]  

    
    def convertToTagDetectableImage(self, image):
        grayFrame = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        grayFrame = cv2.equalizeHist(grayFrame) #Boost contrast
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        grayFrame = clahe.apply(grayFrame)
        grayFrame = self.adjust_gamma(grayFrame)
        return grayFrame


        
    def openCamera(self):
        """
        Opens the camera using the specified camera ID.
        Checks if the camera stream is available; if not, prints an error and exits.
        """
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
            #print(centers[id])
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
        #print(f"p0x = {point0[0]}, p1x = {point1[0]}")
        if point0[0] - point1[0] == 0:
            return 0
        return (point0[1] - point1[1])/(point0[0] - point1[0])

    #logic for setting the boundaries of both the outer chess board and each square within the board
    #inputs might be off
    def get_chessboard_boundaries(self,detections):
        if detections is None:
            return None
        
        idCounter = 0
        for d in detections:
            id = d.tag_id
            if id == 0 or id == 1 or id == 2 or id == 3:
                idCounter += 1
        #print(idCounter)
        if idCounter != 4:
            return None
        
        #TL = top left apriltag, BR = bottom right apriltag, BL = bottom left apriltag ...
        
        TL = detections[3].corners
        BL= detections[0].corners        
        BR = detections[1].corners
        TR = detections[2].corners
        
        TL = [int(TL[2][0]),int(TL[2][1])]
        BL = [int(BL[1][0]),int(BL[1][1])]
        BR = [int(BR[0][0]),int(BR[0][1])]
        TR = [int(TR[3][0]), int(TR[3][1])]
        return [TL, BL, BR, TR]
    
    def getHomoGraphicAppliedImage(self,sourceImage,fourPoints, refImg = None, ow = 900, oh = 800):
        if not refImg:
            refImg = self.referenceImage
        
        refdi = cv2.cvtColor(refImg,cv2.COLOR_BGR2GRAY)
        refdi = cv2.resize(refdi, (0, 0), fx = 0.4, fy = 0.4) #0.4
        refImgDetections = self.aprilDetector.detect(img=refdi)
        #cv2.imshow(f"warped",refdi)
        #return None
        if not refImgDetections:
            return None
        
        refImgPoints = self.get_chessboard_boundaries(refImgDetections)
        
        if not refImgPoints:
            return None
        
        #cv2.imshow(f"warped",refdi)
        source = np.float32(fourPoints)
        ref = np.float32(refImgPoints)
        #print(f"source == {source}")
        mat = cv2.getPerspectiveTransform(source,ref)
        return cv2.warpPerspective(sourceImage,mat,(ow,oh))
    
    def getAxesIntervalDots(self,TL,BL,BR):
        width = self.calculateEuclidianDist(BL,BR)
        height = self.calculateEuclidianDist(TL,BL)

        x = np.linspace(0,width,9)
        y = np.linspace(0,height,9)

        # Uses np.meshgrid to create a grid (representing the chessboard squares).
        mx,my = np.meshgrid(x,y)
        
    
        return mx, my

    
    def plotBoardDots(self,frame,mx,my,TL):
        #if self.chessBoardResetCounter == self.chessBoardResetCounterThreshold:
        #        self.chessBoardVertices = []
        counter = 0
        for i in range(my.shape[0]):
            for j in range(mx.shape[1]):
                x = int(mx[i, j]) + TL[0]
                y = int(my[i, j]) + TL[1]
                if not self.hasBeenCalibrated:
                    self.chessBoardVertices.append([x,y])
                else:
                    self.chessBoardVertices[counter][0] = x
                    self.chessBoardVertices[counter][1] = y

                counter += 1

                cv2.circle(frame, (x, y), 3,(0, 0, 255), -1)
        
        #if self.chessBoardResetCounter == self.chessBoardResetCounterThreshold:
        #self.chessBoardCells = []
        #print(len(self.chessBoardVertices))
        skipCounter = 0
        counter = 0
        for index in range(0,71):
            if skipCounter == 8:
                skipCounter = 0
                continue

            if not self.hasBeenCalibrated:
                self.chessBoardCells.append({"TL": [self.chessBoardVertices[index][0],self.chessBoardVertices[index][1]],
                                        "TR": [self.chessBoardVertices[index+1][0],self.chessBoardVertices[index+1][1]],
                                        "BL": [self.chessBoardVertices[index+9][0],self.chessBoardVertices[index+9][1]],
                                        "BR": [self.chessBoardVertices[index+10][0],self.chessBoardVertices[index+10][1]]})
                 
            else:
                self.chessBoardCells[counter]["TL"]=[self.chessBoardVertices[index][0],self.chessBoardVertices[index][1]]
                self.chessBoardCells[counter]["TR"]=[self.chessBoardVertices[index+1][0],self.chessBoardVertices[index+1][1]]
                self.chessBoardCells[counter]["BL"]=[self.chessBoardVertices[index+9][0],self.chessBoardVertices[index+9][1]]
                self.chessBoardCells[counter]["BR"]=[self.chessBoardVertices[index+10][0],self.chessBoardVertices[index+10][1]]
                 
            counter += 1
            skipCounter += 1
            
        if not self.hasBeenCalibrated:
            self.hasBeenCalibrated = True
        
        counter = 0
        for entry in self.chessBoardCells:
            #print(f"cell{counter}")
            #t = entry["BL"]
            #print(f"BL={t}")
            #t = entry["BR"]
            #print(f"BR={t}")
            #t = entry["TL"]
            #print(f"TL={t}")
            #t = entry["TR"]
            #print(f"TR={t}")
            #print()
            counter += 1
        

        #print(len(self.chessBoardCells))
        #self.chessBoardResetCounter = 0
        #print("reset counter disabled")
            
       
    #marks pieces on the currentboard
    def markPieces(self, pieces, caller):
        if len(self.chessBoardCells) > 0 and len(pieces) > 0:
            for c in range(0,len(self.chessBoardCells)):
                marked = False
                for piece in pieces:
                    pieceCenterX, pieceCenterY = piece.center.astype(int)
                    cell = self.chessBoardCells[c]
                    if cell["BL"][0] < pieceCenterX and pieceCenterX < cell["BR"][0] and \
                        cell["BL"][1] < pieceCenterY and cell["TL"][1] > pieceCenterY:

                        if c["TL"][0] < pieceCenterX and pieceCenterX < c["TR"][0] and\
                            c["BL"][1] < pieceCenterY and pieceCenterY < c["TR"][1]:
                            self.currentBoard[c//8][c%8] = caller.pieceMap[piece.tag_id]
                            marked = True
                if not marked:
                    self.currentBoard[c//8][c%8] = '.'

                    
        

       

    def drawLine(self, frame, p0,p1):
        """
        Draws a green line between two given points on the frame.
        Useful for visualizing board boundaries or axes.
        """
        cv2.line(frame, p0, p1, color=(0, 255, 0), thickness=1)


    #Get the april tags responsible for the board corners
    def getChessBoardCorners(self,detections):
        """
        Filters the detected AprilTags to find those with IDs 0, 1, and 2.
        These are assumed to be the corners of the chessboard.
        If fewer than 3 are found, it prints a warning and returns None.
        """
        ret = []
        
        for d in detections:
            if self.mode == "paper" and \
                (d.tag_id == 1 or d.tag_id == 2 or d.tag_id == 3 or d.tag_id == 4)\
                and d.tag_id not in ret:
                    ret.append(d)
            elif self.mode == "block":
                if (d.tag_id == 5 or d.tag_id == 6 or d.tag_id == 7 or d.tag_id == 8)\
                and d.tag_id not in ret:
                    ret.append(d)

         
        if len(ret) != 4:
            print("Failed to detect all four corners, detecting april tags again")
            return None
        #sort the array
        ret = sorted(ret,key=lambda x : x.tag_id)        
        return ret

    def adjust_gamma(self,image, gamma=1.5):
        """
        Adjusts the gamma of the image to correct for lighting.
        Builds a lookup table and applies it to the image using OpenCV's cv2.LUT.
        """
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                        for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)
    
    #for debugging
    def drawAprilTagCorner(self, frame, detections, cornerPos):
        """
        For debugging: draws a circle at a specified corner (cornerPos) of each detected AprilTag.
        This helps to verify the exact locations of tag corners.
        """
        for d in detections:
            cv2.circle(frame,(int(d.corners[cornerPos][0]),int(d.corners[cornerPos][1])),3,(255,0,255),2)

    def drawPieces(self,frame,pieces,callerClass,fp):
        
        for pc in pieces:
            cx,cy = pc.center.astype(int)
            for f in fp:
                if cx <= f[0][0] or cx >= f[3][0] or cy <= f[0][1] or cy >= f[1][1]:
                    continueFlag = True
                    break
            if continueFlag:
                continue

            color = (0,255,0)
            if pc.tag_id >= 20 and pc.tag_id <= 25:
                color = (0,0,255)
            
            cv2.circle(frame,cx,cy,3,color,2)
            cv2.putText(frame,callerClass.pieceMap[pc.tag_id],
                        cx,cy,cv2.FONT_HERSHEY_PLAIN, 
                        2,color,2)
            
        
        

    def drawBordersandDots(self,frame,detections,grayFrame=None):
            if grayFrame is None:
                grayFrame = frame

            centers = self.getCenterPositionofDetection(detections)
            self.drawCenterCircleForTags(frame,centers)
            cbc = self.getChessBoardCorners(detections)
            if cbc:
                corners = self.get_chessboard_boundaries(cbc)
                mx,my = self.getAxesIntervalDots(corners[0],corners[1],corners[2])
                self.plotBoardDots(frame,mx,my,corners[0])
                self.drawLine(frame,corners[0],corners[1])
                self.drawLine(frame,corners[1],corners[2])
                #print("DRAWING")

    # moved to GamePlay.py 
    # def startLoop(self):
    #     while True:
    #         ret,frame = self.cam.read()
            
    #         grayFrame = self.convertToTagDetectableImage(frame)
    #         detections = self.aprilDetector.detect(img=grayFrame)
    #         #self.drawBordersandDots(frame,detections,grayFrame)
            
    #         if detections:
    #             fp = self.get_chessboard_boundaries(detections)
    #             if fp:
    #                 warpedFrame = self.getHomoGraphicAppliedImage(grayFrame,fp)
    #                 if warpedFrame is not None:
    #                     detections = self.aprilDetector.detect(img=warpedFrame)
    #                     warpedFrame = cv2.cvtColor(warpedFrame,cv2.COLOR_GRAY2BGR)
    #                     self.drawBordersandDots(warpedFrame,detections)
    #                     cv2.imshow(f"warped",warpedFrame)


    #         cv2.imshow(f"FEED Cam-ID = {self.camID}",frame)
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break


   

    def destroyCameraFeed(self, destroyAllWindows=True):
        """
        Clean-Up Function
        """
        self.cam.release()
        if destroyAllWindows: # Optionally closes all OpenCV windows.
            cv2.destroyAllWindows()

if __name__ == "__main__":
    #ocr = CameraFeed(1)
    #ocr.openCamera()
    #ocr.startLoop()
    print("CameraFeed.py")
    #ocr.destroyCameraFeed()