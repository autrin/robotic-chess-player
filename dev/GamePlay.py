import FilePathFinder
from CameraFeed import CameraFeedClass
from ChessEngine import ChessEngineClass
import cv2
#bridge between camera and chess engine
class GamePlayClass:
    def __init__(self):
        self.camera = CameraFeedClass(1)
        self.chessEngine = ChessEngineClass()
        #assume robot will always play white by default
        
        #tagID: [pieceType,cellPos]
        self.tagIDWhitePieces = { 
            0 : ['P',8],
            1 : ['P',9],
            2 : ['P',10],
            3 : ['P',11],
            4 : ['P',12],
            5 : ['P',13],
            6 : ['P',14],
            7 : ['P',15],
            8 : ['R',0],
            9 : ['N',1],
            10 : ['B',2],
            11 : ['Q',3],
            12 : ['K',4],
            13 : ['B',5],
            14 : ['N',6],
            15 : ['R',7]
        }
        self.tagIDBlackPieces = { 
            0 : ['p',48],
            1 : ['p',49],
            2 : ['p',50],
            3 : ['p',51],
            4 : ['p',52],
            5 : ['p',53],
            6 : ['p',54],
            7 : ['p',55],
            8 : ['r',56],
            9 : ['n',57],
            10 : ['b',58],
            11 : ['q',59],
            12 : ['k',60],
            13 : ['b',61],
            14 : ['n',62],
            15 : ['r',63]
        }
        self.tagIDToMyPieces = None
        self.tagIDToOppPieces = None


        #self.homographyCorners = None

    def calibrate(self):
        self.camera.openCamera()
        calibrated = False
        while True:
            ret,frame = self.camera.cam.read()
            
            grayFrame = self.camera.convertToTagDetectableImage(frame)
            detections = self.camera.aprilDetector.detect(img=grayFrame)
            #self.drawBordersandDots(frame,detections,grayFrame)
            
            if detections:
                fp = self.camera.get_chessboard_boundaries(detections)
                if fp:
                    warpedFrame = self.camera.getHomoGraphicAppliedImage(grayFrame,fp)
                    if warpedFrame is not None:
                        detections = self.camera.aprilDetector.detect(img=warpedFrame)
                        warpedFrame = cv2.cvtColor(warpedFrame,cv2.COLOR_GRAY2BGR)
                        self.camera.drawBordersandDots(warpedFrame,detections)
                        cv2.imshow(f"hit c to end calibration",warpedFrame)
                        calibrated = True

            cv2.imshow(f"FEED Cam-ID = {self.camera.camID}",frame)
            if (cv2.waitKey(1) & 0xFF == ord('q')) or ((cv2.waitKey(1) & 0xFF == ord('c')) and calibrated):
                break
        self.camera.destroyCameraFeed()


    def play(self):
        
        self.calibrate()
        
        #assume from this point on, the board has been calibrated
        while True:
            whoGoesFirst = input("who goes first? (ai/human)")
            if whoGoesFirst == "ai":
                self.chessEngine.side = "w"
                self.tagIDToMyPieces = self.tagIDWhitePieces
                self.tagIDToOppPieces = self.tagIDBlackPieces
                break
            elif whoGoesFirst == "human":
                self.chessEngine.side = "b"
                self.tagIDToOppPieces = self.tagIDWhitePieces
                self.tagIDToMyPieces = self.tagIDBlackPieces

                break
            else:
                print("invalid input, try again\n")

        #continuous calibration???
        self.camera.openCamera()
        while True:
            ret,frame = self.camera.cam.read()
            
            grayFrame = self.camera.convertToTagDetectableImage(frame)
            #pieceDetections = self.camera.myChessPieceDetector.detect(img=grayFrame)
            cornerDetections = self.camera.aprilDetector.detect(img=grayFrame)
            
            if cornerDetections:
                fp = self.camera.get_chessboard_boundaries(cornerDetections)
                if fp:
                    warpedFrame = self.camera.getHomoGraphicAppliedImage(grayFrame,fp)
                    if warpedFrame is not None:
                        detections = self.camera.aprilDetector.detect(img=warpedFrame)
                        warpedFrame = cv2.cvtColor(warpedFrame,cv2.COLOR_GRAY2BGR)
                        self.camera.drawBordersandDots(warpedFrame,detections)
                        cv2.imshow(f"warped",warpedFrame)
                        


            cv2.imshow(f"FEED Cam-ID = {self.camera.camID}",frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.destroyCameraFeed()

if __name__ == "__main__":
    gp = GamePlayClass()
    gp.play()