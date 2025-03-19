import FilePathFinder
from CameraFeed import CameraFeedClass
from ChessEngine import ChessEngineClass
import cv2
#bridge between camera and chess engine
class GamePlayClass:
    def __init__(self):
        self.camera = CameraFeedClass(1)
        self.chessEngine = ChessEngineClass()
        self.tagIDWhitePieces = { 
            0 : 'P',
            1 : 'P',
            2 : 'P',
            3 : 'P',
            4 : 'P',
            5 : 'P',
            6 : 'P',
            7 : 'P',
            8 : 'R',
            9 : 'N',
            10 : 'B',
            11 : 'Q',
            12 : 'K',
            13 : 'B',
            14 : 'N',
            15 : 'R'
        }
        self.tagIDBlackPieces = { 
            0 : 'p',
            1 : 'p',
            2 : 'p',
            3 : 'p',
            4 : 'p',
            5 : 'p',
            6 : 'p',
            7 : 'p',
            8 : 'r',
            9 : 'n',
            10 : 'b',
            11 : 'q',
            12 : 'k',
            13 : 'b',
            14 : 'n',
            15 : 'r'
        }
        self.tagIDToMyPieces = None
        self.tagIDToOppPieces = None

        if self.chessEngine.side == 'w':
            self.tagIDToMyPieces = self.tagIDWhitePieces
            self.tagIDToOppPieces = self.tagIDBlackPieces
        else:
            self.tagIDToOppPieces = self.tagIDWhitePieces
            self.tagIDToMyPieces = self.tagIDBlackPieces


    def play(self):
        self.camera.openCamera()
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
                        cv2.imshow(f"warped",warpedFrame)
                        


            cv2.imshow(f"FEED Cam-ID = {self.camera.camID}",frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.camera.destroyCameraFeed()

if __name__ == "__main__":
    gp = GamePlayClass()
    gp.play()