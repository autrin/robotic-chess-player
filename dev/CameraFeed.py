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

        
    #just changed name from campera to camera
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
    
    #logic for setting the boundaries of both the outer chess board and each square within the board
    #inputs might be off
    #returns a pair of 2 lists- one containing the left edges and one containing the top edges
    def set_chessboard_boundaries(self,detections):
        
        #within the detections[] array, 1,2, and 3 are all just arbitrary. As such, 
        #we will need to fill the indices in with the 3 unique tags that correspond to the edges of the board
        #TL = top left apriltag, TR = top right apriltag, BL = bottom left apriltag
        (TL1,TL2,TL3,TL4) = detections[1].corners
        (TR1,TR2,TR3,TR4) = detections[2].corners
        (BL1,BL2,BL3,BL4) = detections[3].corners
        
        #For corners: It looks like the tags wind counterclockwise, but I could be wrong. 
        #this assumes that the top left of each apriltag is the first corner
        #TL3, TR2, and BL4 are the upper left, upper right, and bottom left points that match to the corners of the chessboard
        #I am assuming the top left pixel of the image is 0,0 : x,y

        boardTopWidth = TL3[0]-TR2[0] #width of the chess board in pixels(for use with calculating pixel locations of chess squares)

        #renaming for ease of understanding
        boardTopLeft = TL3
        boardTopRight = TR2
        boardBotLeft = BL4

        chessSquareWidth = boardTopWidth/8 #should be int
        #these next 8 x values are for each LEFT SIDE of the squares on the board,
        #starting at the left edge of the chess board(1) and ending at the left edge of the rightmost square of the chess board(8)
        leftEdge1 = TL3[0]
        leftEdge2 = TL3[0] + chessSquareWidth
        leftEdge3 = TL3[0] + chessSquareWidth*2
        leftEdge4 = TL3[0] + chessSquareWidth*3
        leftEdge5 = TL3[0] + chessSquareWidth*4
        leftEdge6 = TL3[0] + chessSquareWidth*5
        leftEdge7 = TL3[0] + chessSquareWidth*6
        leftEdge8 = TL3[0] + chessSquareWidth*7

        #these next 8 y values are for the TOP SIDE of the squares on the board,
        #starting at the top edge of the chess board(1) and ending at the upper edge of the bottommost square of the chessboard(8)
        topEdge1 = TL3[1]
        topEdge2 = TL3[1] + chessSquareWidth
        topEdge3 = TL3[1] + chessSquareWidth*2
        topEdge4 = TL3[1] + chessSquareWidth*3
        topEdge5 = TL3[1] + chessSquareWidth*4
        topEdge6 = TL3[1] + chessSquareWidth*5
        topEdge7 = TL3[1] + chessSquareWidth*6
        topEdge8 = TL3[1] + chessSquareWidth*7

        return ([leftEdge1,leftEdge2,leftEdge3,leftEdge4,leftEdge5,leftEdge6,leftEdge7,leftEdge8][topEdge1,topEdge2,topEdge3,topEdge4,topEdge5,topEdge6,topEdge7])


        

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
    ocr.openCamera()
    ocr.startLoop()
    
    ocr.destroyCameraFeed()
    