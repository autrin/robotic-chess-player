import FilePathFinder
from CameraFeed import CameraFeedClass
from ChessEngine import ChessEngineClass
import cv2
import mss
import time
import numpy as np
#bridge between camera and chess engine
class GamePlayClass:
    def __init__(self):
        self.camera = CameraFeedClass(1)
        self.chessEngine = ChessEngineClass()
        #assume robot will always play white by default
        
        #tagID: [pieceType,cellPos] pieceType == "x" if captured

        self.pieceMap = { 
            #Black
            20 : 'p',
            21 : 'k',
            22 : 'q',
            23 : 'r',
            24 : 'b',
            25 : 'n',

            #White
            10 : 'P',
            11 : 'K',
            12 : 'Q',
            13 : 'R',
            14 : 'B',
            15 : 'N',
        }

        # self.tagIDWhitePieces = { 
        #     10 : 'P',
        #     11 : 'K',
        #     12 : 'Q',
        #     13 : 'R',
        #     14 : 'B',
        #     15 : 'N',
        # }
        

        
        
        self.turn = "ai"
        self.myPieceDetections = self.tagIDWhitePieces
        self.oppPieceDetections = self.tagIDBlackPieces
        self.HTfp = None
    
    def chessCellPosToCellPos(self,st):
        rank = (int(st[1])-1)*10
        col = ord(st[0])
        return rank + col 

    def cellPosToChessCellPos(self,num):
        col = num%8
        col = chr(ord('a') + col)
        rank = str(8 - num//8)
        return col+rank

    def setCurrboard2PrevBoard(self):
        for i in range(0,8):
            for j in range(0,8):
                self.camera.previosBoard[i][j] = self.camera.currentBoard[i][j]
    #finds move 
    def getmovestr(self,board_before, board_after):
        source = None
        destination = None
        for i in range(8):
            for j in range(8):
                if board_before[i][j] != board_after[i][j]:
                    if board_before[i][j] != '.' and board_after[i][j] == '.':
                        source = (i, j)
                    else:
                        destination = (i,j)

                #does not consider special moves
                if source is not None and destination is not None:
                    break
                    
        if source is None or destination is None:
            return None
        
        
        sourceCell = source[0]*8 + source[1]
        destCell = destination[0]*8 + destination[1]
        source_alg = self.cellPosToChessCellPos(sourceCell)
        dest_alg = self.cellPosToChessCellPos(destCell)
        
        return source_alg + dest_alg

   

    def calibratePieces(self,myPieces, oppPieces):
        self.getMyMoveFromVisual(myPieces)
        self.getOppMoveFromVisual(oppPieces)

    #returns whitepieces, blackpieces
    def getWhiteBlackPieces(self,detections):
        dw = []
        db = []
        for d in detections:
            if d.tag_id >= 4 and d.tag_id <= 9:
                dw.append(d)
            else:
                db.append(d)
        return dw, db
    


    def calibrate(self,computerScreen):
        self.camera.openCamera()
        calibrated = False
        sct = None
        #moniotr = None 
        region = None
       
        if computerScreen:
            sct = mss.mss()
            #monitor = sct.monitors[1]
            region = {"top": 0, "left": 600, "width": 1200, "height": 1200}


        while True:
            frame = None
            if not computerScreen:
                ret,frame = self.camera.cam.read() #for webcam
            else:
                screenshot = sct.grab(region)
                frame = np.array(screenshot)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            grayFrame = None
            if not computerScreen:
                grayFrame = self.camera.convertToTagDetectableImage(frame)
            else: #computer screen, no need for 
                grayFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            
            #grayFrame = self.camera.convertToTagDetectableImage(frame)
            detections = self.camera.aprilDetector.detect(img=grayFrame)
            #self.drawBordersandDots(frame,detections,grayFrame)
            
            if detections:
                self.HTfp = self.camera.get_chessboard_boundaries(detections)
                
                if self.HTfp:
                    warpedFrame = self.camera.getHomoGraphicAppliedImage(grayFrame,self.HTfp)
                    
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
                self.turn = "ai"
                break
            elif whoGoesFirst == "human":
                self.chessEngine.side = "b"
                self.turn = "human"
                break
            else:
                print("invalid input, try again\n")

    def mask4Corners(self, gfCopy,cornerDetections):
        cv2.rectangle(gfCopy, (int(cornerDetections[0].corners[0][0]), int(cornerDetections[0].corners[0][1])), 
                                  (int(cornerDetections[0].corners[2][0]), int(cornerDetections[0].corners[2][1])), (255,0,0), -1)           
        cv2.rectangle(gfCopy, (int(cornerDetections[1].corners[0][0]), int(cornerDetections[1].corners[0][1])), 
                        (int(cornerDetections[1].corners[2][0]), int(cornerDetections[1].corners[2][1])), (255,0,0), -1)
        cv2.rectangle(gfCopy, (int(cornerDetections[2].corners[0][0]), int(cornerDetections[2].corners[0][1])), 
                        (int(cornerDetections[2].corners[2][0]), int(cornerDetections[2].corners[2][1])), (255,0,0), -1)
        cv2.rectangle(gfCopy, (int(cornerDetections[3].corners[0][0]), int(cornerDetections[3].corners[0][1])), 
                        (int(cornerDetections[3].corners[2][0]), int(cornerDetections[3].corners[2][1])), (255,0,0), -1)

    
    def play(self,computerScreen=False):
        #self.calibrate(computerScreen)
        self.determine_side()
        self.camera.openCamera()
        aiMoved = False
        #oppMoved = None
        move = None
        sct = None
        #moniotr = None 
        region = None
        byPass = False
       
        if computerScreen:
            sct = mss.mss()
            #monitor = sct.monitors[1]
            region = {"top": 0, "left": 600, "width": 1200, "height": 1200}

        while True:
            frame = None
            if not computerScreen:
                ret,frame = self.camera.cam.read() #for webcam
            else:
                screenshot = sct.grab(region)
                frame = np.array(screenshot)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            grayFrame = None
            #if not computerScreen:
            #    grayFrame = self.camera.convertToTagDetectableImage(frame)
            #else: #computer screen, no need for 
            grayFrame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            
            cornerDetections = self.camera.aprilDetector.detect(img=grayFrame)
            
            
            if cornerDetections:
                fp = self.camera.get_chessboard_boundaries(cornerDetections)
                if fp:
                    gfCopy = grayFrame.copy()
                    self.mask4Corners(gfCopy,cornerDetections)
                    warpedFrame = self.camera.getHomoGraphicAppliedImage(grayFrame,fp)
                    wfCopy = self.camera.getHomoGraphicAppliedImage(gfCopy,fp)
                    
                    if warpedFrame is not None:
                        cornerDetections2 = self.camera.aprilDetector.detect(img=warpedFrame)
                        fp2 = self.camera.get_chessboard_boundaries(cornerDetections2)
                        
                        if fp2:
                            self.camera.drawBordersandDots(warpedFrame,cornerDetections2)
                            
                            pieces = self.camera.PieceDetector.detect(img=wfCopy)
                            
                            wfCopy = cv2.cvtColor(wfCopy,cv2.COLOR_GRAY2BGR)
                            if pieces:
                                self.camera.markPieces(pieces,self)
                                self.camera.drawPieces(wfCopy,pieces,self,fp2)
                            
                                if self.turn == "ai":
                                    byPass = False #human move needs to be captured after ai's move
                                    if not aiMoved:
                                        # print(self.chessEngine.FEN)
                                        move = self.chessEngine.makeAIMove()
                                        aiMoved = True
                                    else:
                                        print("AI's desired move: " + move)
                                        input("press enter after robot makes the move")

                                    
                                    moveFromVisual = self.getmovestr(self.camera.previosBoard, self.camera.currentBoard)
                                    if moveFromVisual is not None and move == moveFromVisual: #add promotion rule as well?
                                        aiMoved = False
                                        self.turn = "human"
                                        print("AI move validated: " + moveFromVisual)
                                        move = None
 
                                       
                                elif self.turn == "human": 
                                    
                                    move = self.self.getmovestr(self.camera.previosBoard, self.camera.currentBoard)
                                    if not byPass:
                                        input("press enter to register opp move")
                                        byPass = True
                                        
                                    if move is not None:
                                        self.chessEngine.makeOppMove(move)
                                        print(self.chessEngine.FEN)
                                        #self.markCaptured("human",move[2:])
                                        self.turn = "ai"
                                        print("opp move: " + move)
                                        #time.sleep(1)
                                        move = None
                            
                                cv2.imshow(f"warped",wfCopy)
                        cv2.imshow(f"wf",warpedFrame)                
                #cv2.imshow(f"FEED Cam-ID = {self.camera.camID}",frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cv2.imshow(f"FEED Cam-ID = {self.camera.camID}",frame)
            
        self.camera.destroyCameraFeed()
        if computerScreen:
            sct.close()

if __name__ == "__main__":
    gp = GamePlayClass()
    gp.play(True)