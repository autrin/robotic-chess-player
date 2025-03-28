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
        
        #tagID: [pieceType,cellPos] pieceType == "x" if captured
        self.tagIDBlackPieces = { 
            0 : ['p',8],
            1 : ['p',9],
            2 : ['p',10],
            3 : ['p',11],
            4 : ['p',12],
            5 : ['p',13],
            6 : ['p',14],
            7 : ['p',15],
            8 : ['r',0],
            9 : ['n',1],
            10 : ['b',2],
            11 : ['q',3],
            12 : ['k',4],
            13 : ['b',5],
            14 : ['n',6],
            15 : ['r',7]
        }
        self.tagIDWhitePieces = { 
            0 : ['P',48],
            1 : ['P',49],
            2 : ['P',50],
            3 : ['P',51],
            4 : ['P',52],
            5 : ['P',53],
            6 : ['P',54],
            7 : ['P',55],
            8 : ['R',56],
            9 : ['N',57],
            10 : ['B',58],
            11 : ['Q',59],
            12 : ['K',60],
            13 : ['B',61],
            14 : ['N',62],
            15 : ['R',63]
        }
        self.tagIDToMyPieces = None
        self.tagIDToOppPieces = None
        #self.homographyCorners = None
    
    def cellPosToChessCellPos(self,num):
        col = num%8
        col = chr(ord('a') + col)
        rank = str(8 - num//8)
        return col+rank

    #need castling, en-passant, promotion
    #focus on the base cases for now
    def getOppMoveFromVisual(self,oppPieces):
        for op in oppPieces:
            newPos = self.camera.getCellPosofPiece(int(op.center[0]),int(op.center[1]))
            #position has changed, assume only one piece gets to move
            #print(f"opp={op.tag_id}")
            #print(newPos)
            #print([int(op.center[0]),int(op.center[1])])
            #exit()
            if self.tagIDToOppPieces[op.tag_id%16][1] != newPos:
                oldCellString = self.cellPosToChessCellPos(self.tagIDToOppPieces[op.tag_id%16][1])
                newCellString = self.cellPosToChessCellPos(newPos)
                return oldCellString+newCellString
        return None




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
                    #warpedFrame = cv2.resize(warpedFrame, (0, 0), fx = 0.1, fy = 0.1)
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

    def determine_side(self):
        #assume from this point on, the board has been calibrated
        while True:
            #whoGoesFirst = input("who goes first? (ai/human)")
            whoGoesFirst = "ai" #dbg
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


    def play(self):
        
        
        #self.calibrate()
        self.determine_side()

        self.camera.openCamera()
        while True:
            ret,frame = self.camera.cam.read()
            grayFrame = self.camera.convertToTagDetectableImage(frame)
            cornerDetections = self.camera.aprilDetector.detect(img=grayFrame)
            
            #Every programmer's dream: just nest everything
            if cornerDetections:
                fp = self.camera.get_chessboard_boundaries(cornerDetections)
                if fp:
                    gfCopy = grayFrame.copy()
                    

                    cv2.rectangle(gfCopy, (int(cornerDetections[0].corners[0][0]), int(cornerDetections[0].corners[0][1])), 
                                  (int(cornerDetections[0].corners[2][0]), int(cornerDetections[0].corners[2][1])), (255,0,0), -1)
                    
                    cv2.rectangle(gfCopy, (int(cornerDetections[1].corners[0][0]), int(cornerDetections[1].corners[0][1])), 
                                  (int(cornerDetections[1].corners[2][0]), int(cornerDetections[1].corners[2][1])), (255,0,0), -1)
                    cv2.rectangle(gfCopy, (int(cornerDetections[2].corners[0][0]), int(cornerDetections[2].corners[0][1])), 
                                  (int(cornerDetections[2].corners[2][0]), int(cornerDetections[2].corners[2][1])), (255,0,0), -1)
                    cv2.rectangle(gfCopy, (int(cornerDetections[3].corners[0][0]), int(cornerDetections[3].corners[0][1])), 
                                  (int(cornerDetections[3].corners[2][0]), int(cornerDetections[3].corners[2][1])), (255,0,0), -1)
                    

                    warpedFrame = self.camera.getHomoGraphicAppliedImage(grayFrame,fp)
                    wfCopy = self.camera.getHomoGraphicAppliedImage(gfCopy,fp)
                    if warpedFrame is not None:

                        
                        cornerDetections2 = self.camera.aprilDetector.detect(img=warpedFrame)
                        fp2 = self.camera.get_chessboard_boundaries(cornerDetections2)
                        
                        if fp2:
                            self.camera.drawBordersandDots(warpedFrame,cornerDetections2)
                            myPieceDetections = self.camera.myChessPieceDetector.detect(img=wfCopy)
                            oppPieceDetections = self.camera.oppChessPieceDetector.detect(img=wfCopy)

                            warpedFrame = cv2.cvtColor(warpedFrame,cv2.COLOR_GRAY2BGR)
                            if myPieceDetections and oppPieceDetections:
                                self.camera.drawPieces(warpedFrame,oppPieceDetections,myPieceDetections,self,fp)
                                movestr = self.getOppMoveFromVisual(oppPieceDetections)
                                if movestr != None:
                                    print(movestr)
                                    #exit()
                                cv2.imshow(f"warped",warpedFrame)
                                    
                cv2.imshow(f"FEED Cam-ID = {self.camera.camID}",frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.camera.destroyCameraFeed()

if __name__ == "__main__":
    gp = GamePlayClass()
    gp.play()