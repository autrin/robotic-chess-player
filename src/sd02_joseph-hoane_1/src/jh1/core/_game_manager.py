import multiprocessing as mp
import sys
import threading
import time
from typing import Optional, Tuple

import chess
import rospy

from jh1.robotics import Skeleton
from jh1.robotics.robot_ur10e_gripper import RobotUR10eGripper
from jh1.utils.visualize import board_overlay_plot
from jh1.visual import (
    instantiate_detector,
    find_april_tags,
    count_clusters,
    PIECE_TAG_IDS,
    HomographySolver,
)
from jh1.visual.video import WebcamSource
from .ChessMovement import ChessMovementController
from ._engine import Engine
from ._game_state import GameState
from ._orchestrator import Orchestrator
from ..typealias import List2D


class GameManager:
    def __init__(
        self,
        engine_path: str,
        opening_book_path: str,
        engine_min_depth: int,
        engine_max_depth: int,
        engine_elo: int,
        require_verify_engine_move: bool,
        require_move_approval: bool,
        require_move_viz: bool,
        require_board_tag_viz: bool,
    ):
        # Arguments
        self.engine_path: str = engine_path
        self.opening_book_path: str = opening_book_path
        self.engine_min_depth: int = engine_min_depth
        self.engine_max_depth: int = engine_max_depth
        self.engine_elo: int = engine_elo
        self.require_verify_engine_move: bool = require_verify_engine_move
        self.require_move_approval: bool = require_move_approval
        self.require_move_viz: bool = require_move_viz
        self.require_board_tag_viz: bool = require_board_tag_viz

        self.engine_is_white = False
        self.engine: Optional[Engine] = None
        self.game: Optional[GameState] = None
        self.robot: Optional[ChessMovementController] = None
        self.cam: Optional[WebcamSource] = None
        self.detector = None

        # Threading for robot move execution
        self.ai_move: Optional[Tuple[str, bool, bool]] = None
        self.move_executed = threading.Event()
        self.robot_running = False
        self.robot_thread: Optional[threading.Thread] = None

    def robot_thread_func(self):
        """
        Background thread to execute moves on the robot whenever ai_move is set.
        """
        rospy.loginfo("Robot thread started")
        while self.robot_running:
            if self.ai_move:
                uci, is_cap, is_ep = self.ai_move
                success = self.robot.execute_move(uci, is_cap, is_ep)
                if not success:
                    rospy.logerr(f"Robot failed move: {uci}")
                # Signal completion
                self.ai_move = None
                self.move_executed.set()
            time.sleep(0.1)
        rospy.loginfo("Robot thread shutting down")

    def cleanup(self):
        """
        Stop the robot thread and release camera resources.
        """
        self.robot_running = False
        if self.robot_thread and self.robot_thread.is_alive():
            self.robot_thread.join(timeout=2)
        if self.cam:
            self.cam.release()
        print("Resources cleaned up.")

    def capture_board_state(self) -> Tuple:
        """
        Capture an image, detect AprilTags, and compute board homography.
        Returns (img, solver, board_bins, certainty_grid).
        """
        print("Reading frame...")
        img = self.cam.read_frame()
        print("Detecting tags")
        detections = find_april_tags(img, self.detector)
        print("Computing tag clusters")
        clusters = count_clusters(detections)

        # Separate corner tags and piece tags
        corners = [None] * 4
        pieces = []
        for c in clusters:
            if c.is_piece_tag():
                pieces.append(c)
            else:
                idx = c.get_vert_idx_if_corner()
                if idx is not None:
                    corners[idx] = c

        if any(x is None for x in corners):
            print("\nWARNING: Missing corner tags. Adjust board position.")
            return None, None, None, None

        solver = HomographySolver(corners)
        board_bins = solver.bin_pieces(pieces)
        grid = HomographySolver.get_certainty_grid(board_bins)

        if self.require_board_tag_viz:
            mp.Process(target=board_overlay_plot, args=(img, solver, board_bins, grid)).start()
        else:
            print("Skipping board tag visualization...")

        return img, solver, board_bins, grid

    # noinspection PyTypeChecker,PyInconsistentReturns
    # TODO: This is where we can integrate the UI's confirm turn action
    def prompt(self) -> Tuple[str, list]:
        """
        Prompt user to move a piece on the real board, then detect and return the UCI move.
        """
        while True:
            # Right here:
            while True:
                cmd = input("Press Enter after move has been made...\n>> ").strip().lower()

                # TODO: temporary only, replace with UI
                if cmd == "pgn":
                    print(self.game.get_pgn())
                else:
                    break

            img, solver, bins, grid = self.capture_board_state()
            if img is None:
                continue

            # Overlay visualization in subprocess

            print("Current board:")
            self.game.print_board()
            print("Detected new board:")
            new_board = GameState.build_board(bins)
            print(GameState.prettify(new_board))

            found_move = GameState.get_move_diff(self.game, new_board)
            if found_move:
                return found_move, new_board

            print("No move or illegal move detected. Adjust and try again.")

    def verify_move(self, expected_move: str):
        """
        Ask user to re-scan until the detected move matches expected_move.
        """
        while True:
            detected_move, _ = self.prompt()
            if detected_move == expected_move:
                print("Engine move verified.")
                return
            print(f"Mismatch: expected {expected_move}, detected {detected_move}. Try again.")

    def set_robot_move(self, uci: str, is_capture: bool, is_en_passant: bool = False) -> bool:
        """
        Schedule a robot move and wait up to timeout for execution.
        """
        self.ai_move = (uci, is_capture, is_en_passant)
        self.move_executed.clear()

        if self.move_executed.wait(timeout=100):
            return True

        rospy.logerr("Robot movement timed out")
        return False

    def handle_engine_move(self):
        """
        Ask engine for a move, execute it on robot, (optionally verify), and update game state.
        """
        move = self.game.get_engine_move()
        if move is None:  # Checkmate
            return

        algebraic = self.game.get_algebraic(move)
        before = self.game.board.copy()
        is_cap, is_ep = GameState.classify_move(move, before)

        print(f"\nEngine move: {move} ({algebraic})")

        if not self.set_robot_move(move, is_cap, is_ep):
            print("Robot failed to execute engine move")
            return

        print("Robot completed the engine's move")
        if self.require_verify_engine_move:
            self.verify_move(move)

        self.game.offer_move(move, by_white=self.engine_is_white)
        print(f"\nExpected current board configuration (after engine's {algebraic}):")
        self.dump_game_state()
        time.sleep(1)

    def test_loop(self):
        """
        Test mode: text-based moves without vision.
        """
        rospy.loginfo("Entering TEST mode")
        # If engine is white, play first
        if self.engine_is_white:
            self.handle_engine_move()

        while not self.game.board.is_game_over() and not rospy.is_shutdown():
            human_move = input("\nEnter your move (e2e4) or 'quit':\n>> ").strip().lower()
            if human_move == 'quit':
                break

            before = self.game.board.copy()
            if self.game.offer_move(human_move):
                is_cap, is_ep = GameState.classify_move(human_move, before)
                if self.set_robot_move(human_move, is_cap, is_ep):
                    self.game.print_board()
                    print(f"FEN: {self.game.get_fen()}")
                else:
                    self.game.board = before
                    rospy.logerr("Robot failed human move")
                    continue
            else:
                print("Illegal move; try again.")
                continue

            # Engine's reply
            if self.game.is_engine_turn():
                self.handle_engine_move()

        print("\n\nGame over.\n")
        time.sleep(3)
        print(self.game.get_pgn())
        print()

    def full_loop(self):
        """
        Full vision-based game loop.
        """
        rospy.loginfo("Entering FULL mode")
        self.cam = WebcamSource(cam_id=0)
        self.detector = instantiate_detector()

        if self.game.is_engine_turn():
            self.handle_engine_move()

        print("-" * 60)
        print("Human player's turn.\n")
        while not self.game.board.is_game_over():
            # Adjust search depth at start of double-ply
            self.engine.set_depth(
                depth=Engine.get_adaptive_depth(
                    num_pieces=len(self.game.board.piece_map()),
                    min_depth=self.engine_min_depth,
                    max_depth=self.engine_max_depth
                )
            )

            move, _ = self.prompt()
            self.game.board.copy()
            if self.game.offer_move(move):
                print(f"\nDetected move: {move}")
                self.dump_game_state()
            else:
                print("Move rejected; try again.")
                continue

            if self.game.is_engine_turn():
                self.handle_engine_move()

            print("-" * 60)
            print("Human player's turn.\n")

        print("\n\nGame over.\n")
        time.sleep(3)
        print(self.game.get_pgn())
        print()

    def dump_game_state(self):
        self.game.print_board()
        print(f"Stockfish evaluation (depth {self.engine.current_depth}): ", end="")
        print(self.engine.get_eval_score())

    def run(self):
        """
        Initialize engine, robot, and start appropriate game loop.
        """
        # Choose sides

        starting_fen: str = chess.STARTING_FEN

        res = input(
            f"\nBegin new game from scratch (Enter) "
            f"or input FEN string to start from."
            f"\n>> "
        ).strip()

        if len(res) != 0:
            try:
                print(GameState.prettify(chess.Board(res)))  # See if the FEN string is valid
                starting_fen = res
            except ValueError:
                print("Invalid input, starting from new board")
                pass

        self.engine_is_white = input(
            "\nShould the engine play as White? (y/n): "
        ).strip().lower() == 'y'

        # Init engine and game state
        self.engine = Engine(
            engine_path=self.engine_path,
            search_depth=self.engine_min_depth,
            force_elo=self.engine_elo,
            opening_book_path=self.opening_book_path
        )
        self.engine.set_position(starting_fen)

        # Must init game after engine
        self.game = GameState(self.engine, engine_plays_white=self.engine_is_white)
        self.game.set_fen(starting_fen)

        # Set depth to an appropriate level for the current board
        self.engine.set_depth(
            depth=Engine.get_adaptive_depth(
                num_pieces=len(self.game.board.piece_map()),
                min_depth=self.engine_min_depth,
                max_depth=self.engine_max_depth
            )
        )

        print(f"Engine is set to {self.engine_elo} ELO, current depth {self.engine.current_depth}")

        # Setup robot controller
        orch = Orchestrator(
            skeleton=Skeleton(RobotUR10eGripper(is_gripper_up=True)),
            require_viz=self.require_move_viz,
            require_approval=self.require_move_approval
        )

        self.robot = ChessMovementController(
            orchestrator=orch,
            simulation_mode=rospy.get_param('sim', True),
            robot_is_white=self.engine_is_white
        )

        # Start robot thread
        self.robot_running = True
        self.robot_thread = threading.Thread(target=self.robot_thread_func)
        self.robot_thread.daemon = True
        self.robot_thread.start()

        # Dispatch to test or full loop
        test_mode = len(sys.argv) > 1 and sys.argv[1] == 'test'
        if test_mode:
            self.test_loop()
        else:
            self.full_loop()
