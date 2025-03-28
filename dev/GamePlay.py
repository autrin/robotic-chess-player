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
        # assume robot will always play white by default

        # tagID: [pieceType,cellPos] pieceType == "x" if captured
        self.tagIDWhitePieces = {
            0: ['P', 8],
            1: ['P', 9],
            2: ['P', 10],
            3: ['P', 11],
            4: ['P', 12],
            5: ['P', 13],
            6: ['P', 14],
            7: ['P', 15],
            8: ['R', 0],
            9: ['N', 1],
            10: ['B', 2],
            11: ['Q', 3],
            12: ['K', 4],
            13: ['B', 5],
            14: ['N', 6],
            15: ['R', 7]
        }
        self.tagIDBlackPieces = {
            0: ['p', 48],
            1: ['p', 49],
            2: ['p', 50],
            3: ['p', 51],
            4: ['p', 52],
            5: ['p', 53],
            6: ['p', 54],
            7: ['p', 55],
            8: ['r', 56],
            9: ['n', 57],
            10: ['b', 58],
            11: ['q', 59],
            12: ['k', 60],
            13: ['b', 61],
            14: ['n', 62],
            15: ['r', 63]
        }
        self.tagIDToMyPieces = None
        self.tagIDToOppPieces = None
        # self.homographyCorners = None

    def cellPosToChessCellPos(self, num):
        rank = num//8
        rank = chr(ord('a') + rank)
        col = str(num % 8)
        return rank+col

    # need castling, en-passant, promotion
    # focus on the base cases for now
    def getOppMoveFromVisual(self, oppPieces):
        for op in oppPieces:
            newPos = self.camera.getCellPosofPiece(
                int(op.center[0]), int(op.center[1]))
            # position has changed, assume only one piece gets to move
            if self.tagIDToOppPieces[op.tag_id][1] != newPos:
                oldCellString = self.cellPosToChessCellPos(
                    self.tagIDToOppPieces[op.tag_id][1])
                newCellString = self.cellPosToChessCellPos(newPos)
                return oldCellString+newCellString

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

    def play(self):

        self.calibrate()
        self.determine_side()

        self.camera.openCamera()
        while True:
            ret, frame = self.camera.cam.read()
            grayFrame = self.camera.convertToTagDetectableImage(frame)
            cornerDetections = self.camera.aprilDetector.detect(img=grayFrame)

            # Every programmer's dream: just nest everything
            if cornerDetections:
                fp = self.camera.get_chessboard_boundaries(cornerDetections)
                if fp:
                    warpedFrame = self.camera.getHomoGraphicAppliedImage(
                        grayFrame, fp)
                    if warpedFrame is not None:
                        myPieceDetections = self.camera.myChessPieceDetector.detect(
                            img=warpedFrame)
                        oppPieceDetections = self.camera.oppChessPieceDetector.detect(
                            img=warpedFrame)
                        if myPieceDetections and oppPieceDetections:

                            cornerDetections = self.camera.aprilDetector.detect(
                                img=warpedFrame)
                            warpedFrame = cv2.cvtColor(
                                warpedFrame, cv2.COLOR_GRAY2BGR)
                            self.camera.drawBordersandDots(
                                warpedFrame, cornerDetections)
                            self.camera.drawPieces(
                                warpedFrame, oppPieceDetections, myPieceDetections, self)
                            cv2.imshow(f"warped", warpedFrame)

                cv2.imshow(f"FEED Cam-ID = {self.camera.camID}", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.camera.destroyCameraFeed()

    def getFENStringFromVision(self):
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
