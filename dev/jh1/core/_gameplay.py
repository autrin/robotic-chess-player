from typing import Optional, Dict

import cv2
import numpy as np

from ._board_state import BoardState
from ._engine import Engine
from jh1.visual import ImageProcessor
from jh1.ui import GameUI
from jh1.visual.video import AbstractVideoSource
from jh1.visual.video._legacy_camera_feed import LegacyCameraFeedClass


class Gameplay:
    def __init__(
        self,
        engine_path: str,
        video_source,
        piece_map: Dict[int, str]
    ):
        self.engine: Engine = Engine(engine_path)
        self.vision: ImageProcessor = ImageProcessor()
        self.ui: GameUI = GameUI()
        self.board: BoardState = BoardState()
        self.source = video_source
        self.piece_map: Dict[int, str] = piece_map
        self.turn: str = "ai"
        self.ai_moved: bool = False
        self.move: Optional[str] = None

        self.camera = LegacyCameraFeedClass(mode="block")

    def loop(self):
        while True:
            frame = self.source.read()[1]
            if frame is None:
                continue

            gray = self.camera.convertToTagDetectableImage(frame)
            corner_detections = self.camera.aprilDetector.detect(gray)

            if corner_detections:
                fp = self.camera.get_chessboard_boundaries(corner_detections)
                if fp:
                    gf_copy = gray.copy()
                    warped = self.camera.getHomoGraphicAppliedImage(gray, fp)
                    masked = self.camera.getHomoGraphicAppliedImage(gf_copy, fp)

                    if warped is not None:
                        tag_detections = self.camera.aprilDetector.detect(warped)
                        warped_bgr = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
                        fp2 = self.camera.get_chessboard_boundaries(tag_detections)

                        if fp2:
                            pieces = self.camera.PieceDetector.detect(masked)
                            if pieces:
                                self.camera.markPieces(pieces, self)
                                self.camera.drawPieces(masked, pieces, self, fp2)

                                if self.turn == "ai":
                                    if not self.ai_moved:
                                        self.move = self.solver.get_best_move()
                                        print("AI move:", self.move)
                                        input("Press Enter when AI has moved")
                                        self.ai_moved = True
                                    else:
                                        visual_move = self.board.get_move()
                                        if visual_move == self.move:
                                            print("AI move validated")
                                            self.board.sync()
                                            self.turn = "human"
                                            self.ai_moved = False
                                elif self.turn == "human":
                                    input("Press Enter after human move")
                                    visual_move = self.board.get_move()
                                    if visual_move:
                                        self.solver.make_opponent_move(visual_move)
                                        print("Human move:", visual_move)
                                        self.board.sync()
                                        self.turn = "ai"

                                self.ui.show_frame("Board", masked)
                        self.ui.show_frame("Warped", warped_bgr)

            self.ui.show_frame("Raw", frame)
            if self.ui.wait_quit():
                break

        self.source.release()
        self.ui.destroy()
