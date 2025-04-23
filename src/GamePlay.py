import time
import rospy
from FilePathFinder import getPath
from CameraFeed import CameraFeedClass
from ChessEngine import ChessEngineClass
from ChessMovement import ChessMovementController
import cv2
import mss
import numpy as np
import threading

class GamePlayClass:
    def __init__(self):
        # Initialize ROS node if it hasn't been done already
        try:
            if not rospy.get_node_uri():
                rospy.init_node('chess_game', anonymous=True, disable_signals=True)
        except:
            rospy.init_node('chess_game', anonymous=True, disable_signals=True)
            
        self.camera = CameraFeedClass(0)
        
        # Initialize the chess engine with the correct path
        self.chessEngine = ChessEngineClass(enginePath="/usr/games/stockfish")
        
        # Initialize robot movement controller
        self.robot = ChessMovementController()
        
        # Piece mapping
        self.pieceMap = {
            # Black
            20: 'p',
            21: 'r',
            22: 'n',
            23: 'b',
            24: 'q',
            25: 'k',

            # White
            10: 'P',
            11: 'R',
            12: 'N',
            13: 'B',
            14: 'Q',
            15: 'K',
        }

        self.turn = "ai"
        self.HTfp = None
        self.ai_move = None
        self.running = True
        self.move_executed = threading.Event()
        
    def chessCellPosToCellPos(self, st):
        rank = (int(st[1]) - 1) * 10
        col = ord(st[0])
        return rank + col

    def cellPosToChessCellPos(self, num):
        col = num % 8
        col = chr(ord('a') + col)
        rank = str(8 - num // 8)
        return col + rank

    def setCurrboard2PrevBoard(self):
        for i in range(0, 8):
            for j in range(0, 8):
                self.camera.previosBoard[i][j] = self.camera.currentBoard[i][j]

    # finds move
    def getmovestr(self, board_before, board_after):
        source = None
        destination = None
        for i in range(8):
            breakOuter = False
            for j in range(8):
                if board_before[i][j] != board_after[i][j]:
                    if board_before[i][j] != '.' and board_after[i][j] == '.':
                        source = (i, j)
                    else:
                        destination = (i, j)

                if source is not None and destination is not None:
                    breakOuter = True
                    break
            if breakOuter:
                break

        if source is None or destination is None:
            return None

        sourceCell = source[0] * 8 + source[1]
        destCell = destination[0] * 8 + destination[1]
        source_alg = self.cellPosToChessCellPos(sourceCell)
        dest_alg = self.cellPosToChessCellPos(destCell)

        return source_alg + dest_alg

    # def calibratePieces(self,myPieces, oppPieces):
    # self.getMyMoveFromVisual(myPieces)
    # self.getOppMoveFromVisual(oppPieces)

    # returns whitepieces, blackpieces
    def getWhiteBlackPieces(self, detections):
        dw = []
        db = []
        for d in detections:
            if d.tag_id >= 4 and d.tag_id <= 9:
                dw.append(d)
            else:
                db.append(d)
        return dw, db

    def calibrate(self, computerScreen):
        self.camera.openCamera()
        calibrated = False
        sct = None
        # moniotr = None
        region = None

        if computerScreen:
            sct = mss.mss()
            # monitor = sct.monitors[1]
            region = {"top": 0, "left": 0, "width": 1920, "height": 1080}

        while True:
            frame = None
            if not computerScreen:
                ret, frame = self.camera.cam.read()  # for webcam
            else:
                screenshot = sct.grab(region)
                frame = np.array(screenshot)

            grayFrame = None
            if not computerScreen:
                grayFrame = self.camera.convertToTagDetectableImage(frame)
            else:  # computer screen, no need for
                grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # grayFrame = self.camera.convertToTagDetectableImage(frame)
            detections = self.camera.aprilDetector.detect(grayFrame)

            # self.drawBordersandDots(frame,detections,grayFrame)

            if detections:
                self.HTfp = self.camera.get_chessboard_boundaries(detections)

                if self.HTfp:
                    warpedFrame = self.camera.getHomoGraphicAppliedImage(grayFrame, self.HTfp)

                    if warpedFrame is not None:
                        detections = self.camera.detectPlz(warpedFrame, self.camera.aprilDetector)
                        # self.camera.aprilDetector.detect(img=warpedFrame)
                        warpedFrame = cv2.cvtColor(warpedFrame, cv2.COLOR_GRAY2BGR)
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
            # whoGoesFirst = input("who goes first? (ai/human)")
            whoGoesFirst = "ai"  # dbg
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

    def mask4Corners(self, gfCopy, cornerDetections):

        cv2.rectangle(gfCopy, (int(cornerDetections[0].corners[0][0]), int(cornerDetections[0].corners[0][1])),
                      (int(cornerDetections[0].corners[2][0]), int(cornerDetections[0].corners[2][1])), (255, 0, 0), -1)
        cv2.rectangle(gfCopy, (int(cornerDetections[1].corners[0][0]), int(cornerDetections[1].corners[0][1])),
                      (int(cornerDetections[1].corners[2][0]), int(cornerDetections[1].corners[2][1])), (255, 0, 0), -1)
        cv2.rectangle(gfCopy, (int(cornerDetections[2].corners[0][0]), int(cornerDetections[2].corners[0][1])),
                      (int(cornerDetections[2].corners[2][0]), int(cornerDetections[2].corners[2][1])), (255, 0, 0), -1)
        cv2.rectangle(gfCopy, (int(cornerDetections[3].corners[0][0]), int(cornerDetections[3].corners[0][1])),
                      (int(cornerDetections[3].corners[2][0]), int(cornerDetections[3].corners[2][1])), (255, 0, 0), -1)

    def play(self, computerScreen=False):
        # Start robot movement thread
        robot_thread = threading.Thread(target=self.robot_thread)
        robot_thread.daemon = True
        robot_thread.start()
        
        self.determine_side()
        self.camera.openCamera()

        sct, region = None, None
        if computerScreen:
            sct = mss.mss()
            region = {"top": 0, "left": 0, "width": 1250, "height": 900}

        aiMoved = False
        move = None
        byPass = False

        while self.running:
            # --- FRAME ACQUISITION ---
            frame = self._get_frame_from_source(computerScreen, sct, region)
            gray = self._get_gray_frame_from_source(frame, computerScreen)

            # --- APRILTAG DETECTION ---
            detections = self.camera.detectPlz(gray, self.camera.aprilDetector)
            fp = self.camera.get_chessboard_boundaries(detections)
            if not fp:
                cv2.imshow(f"FEED Cam-ID = {self.camera.camID}", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # --- APPLY HOMOGRAPHY TO FRAME + PIECE CENTERS ---
            warped_gray, H = self.camera.getHomoGraphicAppliedImage(gray, fp)
            if warped_gray is None or H is None:
                continue

            transformed_pieces = self.camera.transform_detections(detections, H)
            self.camera.markPiecesTransformed(transformed_pieces, self)
            # --- DRAW TAG METADATA ON WARPED FRAME ---
            for tag_id, warped_center in transformed_pieces:
                cx, cy = int(warped_center[0]), int(warped_center[1])
                text = f"{tag_id}"
                cv2.putText(warped_gray, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (255, 255, 255), 1, cv2.LINE_AA)

            print("\n".join(["".join(rank) for rank in self.camera.currentBoard]))

            # --- GAME LOGIC ---
            if self.turn == "ai":
                if not aiMoved:
                    move = self.chessEngine.makeAIMove()
                    aiMoved = True
                    move_start_time = time.time()
                    self.ai_move = move  # Store move for robot to execute
                    print("AI's desired move: " + move)
                    # Signal the robot thread to execute the move
                    self.move_executed.clear()
                    # Wait for robot to complete the move
                    if not self.move_executed.wait(timeout=30):
                        print("Robot movement timed out")
                    time.sleep(2)  # Short delay after move completes
                else:
                    print("AI's desired move: " + move)

                # Verify the move was made correctly
                moveFromVisual = self.getmovestr(self.camera.previosBoard, self.camera.currentBoard)
                if moveFromVisual and move == moveFromVisual:
                    print("AI move validated: " + moveFromVisual)
                    aiMoved = False
                    move = None
                    self.turn = "human"
                    self.setCurrboard2PrevBoard()
                    self.camera.resetCounters()
                elif time.time() - move_start_time > 30:
                    print("Move validation timed out, retrying...")
                    aiMoved = False  # Reset to try again
                    
            elif self.turn == "human":
                move = self.getmovestr(self.camera.previosBoard, self.camera.currentBoard)
                if not byPass:
                    input("Press enter to register human move")
                    byPass = True

                if move:
                    print("Human move: " + move)
                    self.chessEngine.makeOppMove(move)
                    print("FEN after human move:", self.chessEngine.FEN)
                    self.turn = "ai"
                    move = None
                    self.setCurrboard2PrevBoard()
                    self.camera.resetCounters()
                    byPass = False

            # --- DISPLAY ---
            display = cv2.cvtColor(warped_gray, cv2.COLOR_GRAY2BGR)
            self.camera.drawBordersandDots(display, detections, grayFrame=warped_gray)
            cv2.imshow("Warped Board View", display)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.camera.destroyCameraFeed()
        if sct:
            sct.close()
    
    def robot_thread(self):
        """Thread that handles robot movement"""
        while self.running:
            if self.ai_move:
                move = self.ai_move
                try:
                    print(f"Robot executing move: {move}")
                    success = self.robot.execute_move(move)
                    if success:
                        print(f"Robot successfully executed move: {move}")
                    else:
                        print(f"Robot failed to execute move: {move}")
                except Exception as e:
                    print(f"Error executing move: {e}")
                finally:
                    self.ai_move = None
                    self.move_executed.set()
            time.sleep(0.1)

    def _get_frame_from_source(self, computerScreen, sct, region):
        if not computerScreen:
            ret, frame = self.camera.cam.read()
            return frame
        else:
            screenshot = sct.grab(region)
            return cv2.cvtColor(np.array(screenshot), cv2.COLOR_BGRA2BGR)

    def _get_gray_frame_from_source(self, frame, computerScreen):
        if not computerScreen:
            return self.camera.convertToTagDetectableImage(frame)
        else:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _get_frame(self, sct, region, computerScreen):
        if not computerScreen:
            ret, frame = self.camera.cam.read()
        else:
            screenshot = sct.grab(region)
            frame = np.array(screenshot)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    def _get_gray_frame(self, frame, computerScreen):
        return (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if computerScreen else self.camera.convertToTagDetectableImage(frame))

    def _check_quit(self):
        return cv2.waitKey(1) & 0xFF == ord('q')

    def _show_frame(self, frame):
        cv2.imshow(f"FEED Cam-ID = {self.camera.camID}", frame)

    def _handle_ai_turn(self, aiMoved, move):
        if not aiMoved:
            move = self.chessEngine.makeAIMove()
            print("AI's desired move: " + move)
            input("press enter after robot makes the move")
        else:
            print("AI's desired move: " + move)

        visualMove = self.getmovestr(self.camera.previosBoard, self.camera.currentBoard)
        if visualMove and move == visualMove:
            self.turn = "human"
            self.setCurrboard2PrevBoard()
            self.camera.resetCounters()
            print("AI move validated: " + visualMove)
            return None
        return move

    def _handle_human_turn(self, byPass):
        move = self.getmovestr(self.camera.previosBoard, self.camera.currentBoard)
        if not byPass:
            input("press enter to register opponent's move")

        if move:
            self.chessEngine.makeOppMove(move)
            print("Opponent move: " + move)
            print(self.chessEngine.FEN)
            self.turn = "ai"
            self.setCurrboard2PrevBoard()
            self.camera.resetCounters()
            return None
        return move


if __name__ == "__main__":
    gp = GamePlayClass()
    gp.play(False)
