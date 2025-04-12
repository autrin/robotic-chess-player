import FilePathFinder
from CameraFeed import CameraFeedClass
from ChessEngine import ChessEngineClass
import cv2
import rospy

# bridge between camera and chess engine

class GamePlayClass:
    def __init__(self):
        self.camera = CameraFeedClass(1)
        self.chessEngine = ChessEngineClass()
        #assume robot will always play white by default
        
        #tagID: [pieceType,cellPos] pieceType == "x" if captured
        self.tagIDBlackPieces = { 
            4 : ['p',8],
            5 : ['p',9],
            6 : ['p',10],
            7 : ['p',11],
            8 : ['p',12],
            9 : ['p',13],
            10 : ['p',14],
            11 : ['p',15],
            12 : ['r',0],
            13 : ['n',1],
            14 : ['b',2],
            15 : ['q',3],
            16 : ['k',4],
            17 : ['b',5],
            18 : ['n',6],
            19 : ['r',7]
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
        self.turn = "ai"
        self.myPieceDetections = None
        self.oppPieceDetections = None
    
    def chessCellPosToCellPos(self,st):
        rank = (int(st[1])-1)*10
        col = ord(st[0])
        return rank + col 

    def cellPosToChessCellPos(self,num):
        col = num%8
        col = chr(ord('a') + col)
        rank = str(8 - num//8)
        return col+rank

    # need castling, en-passant, promotion
    # focus on the base cases for now
    def getOppMoveFromVisual(self, oppPieces):
        for op in oppPieces:
            newPos = self.camera.getCellPosofPiece(int(op.center[0]),int(op.center[1]))
            if self.tagIDToOppPieces[op.tag_id][1] != newPos:
                oldCellString = self.cellPosToChessCellPos(
                    self.tagIDToOppPieces[op.tag_id][1])
                newCellString = self.cellPosToChessCellPos(newPos)
                return oldCellString+newCellString
        return None

    def getMyMoveFromVisual(self,myPieces):
        for mp in myPieces:
            if self.tagIDToMyPieces[mp.tag_id][0] == "x": #ignore captured pieces
                continue
            newPos = self.camera.getCellPosofPiece(int(mp.center[0]),int(mp.center[1]))
            if self.tagIDToMyPieces[mp.tag_id%16][1] != newPos:
                oldCellString = self.cellPosToChessCellPos(self.tagIDToMyPieces[mp.tag_id%16][1])
                newCellString = self.cellPosToChessCellPos(newPos)
                return oldCellString+newCellString
        return None




    def calibrate(self):
        self.camera.openCamera()
        calibrated = False
        while True:
            ret, frame = self.camera.cam.read()

            grayFrame = self.camera.convertToTagDetectableImage(frame)
            detections = self.camera.aprilDetector.detect(img=grayFrame)
            # self.drawBordersandDots(frame,detections,grayFrame)

            if detections:
                fp = self.camera.get_chessboard_boundaries(detections)

                if fp:
                    warpedFrame = self.camera.getHomoGraphicAppliedImage(
                        grayFrame, fp)
                    # warpedFrame = cv2.resize(warpedFrame, (0, 0), fx = 0.1, fy = 0.1)
                    if warpedFrame is not None:
                        detections = self.camera.aprilDetector.detect(
                            img=warpedFrame)
                        warpedFrame = cv2.cvtColor(
                            warpedFrame, cv2.COLOR_GRAY2BGR)
                        self.camera.drawBordersandDots(warpedFrame, detections)
                        cv2.imshow(f"hit c to end calibration", warpedFrame)
                        calibrated = True

            cv2.imshow(f"FEED Cam-ID = {self.camera.camID}", frame)
            if (cv2.waitKey(1) & 0xFF == ord('q')) or ((cv2.waitKey(1) & 0xFF == ord('c')) and calibrated):
                break
        self.camera.destroyCameraFeed()

    def determine_side(self):
        # assume from this point on, the board has been calibrated
        while True:
            #whoGoesFirst = input("who goes first? (ai/human)")
            whoGoesFirst = "ai" #dbg
            if whoGoesFirst == "ai":
                self.chessEngine.side = "w"
                self.tagIDToMyPieces = self.tagIDWhitePieces
                self.tagIDToOppPieces = self.tagIDBlackPieces
                self.turn = "ai"
                self.myPieceDetections = self.camera.whiteChessPieceDetector
                self.oppPieceDetections = self.camera.blackChessPieceDetector
                break
            elif whoGoesFirst == "human":
                self.chessEngine.side = "b"
                self.tagIDToOppPieces = self.tagIDWhitePieces
                self.tagIDToMyPieces = self.tagIDBlackPieces
                self.turn = "human"
                self.oppPieceDetections = self.camera.whiteChessPieceDetector 
                self.myPieceDetections = self.camera.blackChessPieceDetector
                

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

    #is this really needed?
    #en-passant rule
    #Not really needed
    # def markCaptured(self,side,dest):
    #     cellNum = self.chessCellPosToCellPos(dest)
    #     if side == "ai":
    #         ep_square = self.chessEngine.board.ep_square
    #         if ep_square is not None:
    #             cell_name = self.chess.square_name(ep_square)

    #         for k in self.tagIDToOppPieces.keys():
    #             if dest == cell_name and\
    #                self.cellPosToChessCellPos(self.tagIDToOppPieces[k][1])[1:]:

    #             if self.tagIDToOppPieces[k][1] == cellNum:
    #                 self.tagIDToOppPieces[k][0] = "x"
    #                 return
                
            

    #     else:
    #         for k in self.tagIDToMyPieces.keys():
    #             if self.tagIDToMyPieces[k][1] == cellNum:
    #                 self.tagIDToMyPieces[k][0] = "x"
    #                 return

    
    def play(self):
        #self.calibrate()
        self.determine_side()
        self.camera.openCamera()
        aiMoved = False
        oppMoved = None
        move = None
        while True:
            ret, frame = self.camera.cam.read()
            grayFrame = self.camera.convertToTagDetectableImage(frame)
            cornerDetections = self.camera.aprilDetector.detect(img=grayFrame)

            # Every programmer's dream: just nest everything
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
                            myPieceDetections = self.myPieceDetections.detect(img=wfCopy)
                            oppPieceDetections = self.oppPieceDetections.detect(img=wfCopy)

                            warpedFrame = cv2.cvtColor(warpedFrame,cv2.COLOR_GRAY2BGR)
                            if myPieceDetections and oppPieceDetections:
                                self.camera.drawPieces(wfCopy,oppPieceDetections,myPieceDetections,self,fp)
                                
                                if self.turn == "ai":
                                    
                                    if not aiMoved:
                                        move = self.chessEngine.makeAIMove()
                                        aiMoved = True
                                    else:
                                        print("AI's desired move: " + move)
                                    #self.markCaptured("ai",move[2:])
                                    moveFromVisual = self.getMyMoveFromVisual(myPieceDetections)
                                    if moveFromVisual is not None and move == moveFromVisual: #add promotion rule as well?
                                        aiMoved = False
                                        self.turn = "human"
                                        print("AI move validated: " + moveFromVisual)
                                        move = None
                                        
                                        #exit() #will exit if there is a difference (for debugging)
                                
                                elif self.turn == "human": 
                                    #oppMoved = input("type \"a\" after making a move")
                                    move = self.getOppMoveFromVisual(oppPieceDetections)
                                    print("opp move??: " + str(move))
                                    if move is not None:
                                        self.chessEngine.makeOppMove(move)
                                        #self.markCaptured("human",move[2:])
                                        self.turn = "ai"
                                        print("opp move: " + move)
                                        move = None
                            
                                cv2.imshow(f"warped",wfCopy)
                                    
                cv2.imshow(f"FEED Cam-ID = {self.camera.camID}",frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.camera.destroyCameraFeed()

    def getFENStringFromVision(self): # Might not need
        """
        Captures an image from the camera, processes it to detect the chessboard,
        detects pieces in each cell, and returns a FEN string representing the board state.
        """
        # 1. Capture an image from the camera.
        ret, frame = self.camera.cam.read()
        if not ret:
            rospy.logerr("Failed to capture image from camera")
            return None

        # 2. Pre-process the image (convert to grayscale, enhance contrast, apply gamma correction)
        processed_image = self.camera.convertToTagDetectableImage(frame)

        # 3. Detect the chessboard corners using your AprilTag detector.
        board_detections = self.camera.aprilDetector.detect(
            img=processed_image)
        board_corners = self.camera.get_chessboard_boundaries(board_detections)
        if board_corners is None:
            rospy.logwarn("Could not detect board boundaries")
            return None

        # 4. Warp the image so that the board is viewed from above.
        warped_image = self.camera.getHomoGraphicAppliedImage(
            processed_image, board_corners)
        if warped_image is None:
            rospy.logwarn("Failed to warp image")
            return None

        # 5. Divide the warped image into cells.
        # Here we use your existing function to get grid intervals.
        # assuming TL is the top-left corner after warping
        TL = board_corners[0]
        mx, my = self.camera.getAxesIntervalDots(
            TL, board_corners[1], board_corners[2])

        # 6. Create an empty 8x8 board representation.
        board_array = [['' for _ in range(8)] for _ in range(8)]

        # 7. Use your piece detectors to detect pieces in the warped image.
        my_piece_detections = self.camera.myChessPieceDetector.detect(
            img=warped_image)
        opp_piece_detections = self.camera.oppChessPieceDetector.detect(
            img=warped_image)

        # 8. For each detection, determine which cell it falls into.
        #    (we have getCellPosofPiece(x, y), that returns a cell index 0..63.)
        for det in my_piece_detections:
            cell_index = self.camera.getCellPosofPiece(
                det.center[0], det.center[1])
            if cell_index != -1:
                row = cell_index // 8
                col = cell_index % 8
                # Use your mapping dictionary for your own pieces.
                piece_type = self.tagIDToMyPieces[det.tag_id % 16][0]
                board_array[row][col] = piece_type

        for det in opp_piece_detections:
            cell_index = self.camera.getCellPosofPiece(
                det.center[0], det.center[1])
            if cell_index != -1:
                row = cell_index // 8
                col = cell_index % 8
                # Use your mapping dictionary for opponent pieces.
                piece_type = self.tagIDToOppPieces[det.tag_id % 16][0]
                board_array[row][col] = piece_type

        # 9. Convert the 8x8 board into FEN format.
        fen_rows = []
        for row in board_array:
            fen_row = ""
            empty_count = 0
            for cell in row:
                if cell == "":
                    empty_count += 1
                else:
                    if empty_count > 0:
                        fen_row += str(empty_count)
                        empty_count = 0
                    fen_row += cell
            if empty_count > 0:
                fen_row += str(empty_count)
            fen_rows.append(fen_row)
        fen_board = "/".join(fen_rows)

        # 10. Assemble the complete FEN string.
        # Here we assume it's white's turn, with full castling rights, no en passant, and move counters set to defaults.
        fen_string = f"{fen_board} w KQkq - 0 1"
        return fen_string


if __name__ == "__main__":
    gp = GamePlayClass()
    gp.play()
