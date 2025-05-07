#! /usr/bin/env python3
import os
import sys
import threading
import time

import chess
import rospy

from jh1.core import Engine, GameState, ChessMovementController, Orchestrator
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

"""
A standalone vision-based chess system
"""
# If resources are in a parent directory named 'resources'
resources_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             "resources")
ENGINE_PATH = "/usr/games/stockfish"
OPENING_BOOK_PATH = os.path.join(resources_dir, "baron30.bin")

# Global variables for thread synchronization
move_executed = threading.Event()
ai_move = None
robot_running = True


def capture_board_state(cam, detector):
    print("Reading frame...")
    img = cam.read_frame()
    print("Detecting tags")
    detections = find_april_tags(img, detector)
    print("Computing tag clusters")
    clusters = count_clusters(detections)
    print(f"{len(clusters)} tags detected")

    corners_tags = [None] * 4
    pieces_tags = []
    for cluster in clusters:
        if cluster.is_piece_tag():
            pieces_tags.append(cluster)
        else:
            idx = cluster.get_vert_idx_if_corner()
            if idx is not None:
                corners_tags[idx] = cluster

    if any(x is None for x in corners_tags):
        print("WARNING: Missing corner tags. Adjust board position.")
        return None, None, None, None

    solver = HomographySolver(corners_tags)
    board_bins = solver.bin_pieces(pieces_tags)
    certainty_grid = HomographySolver.get_certainty_grid(board_bins)
    return img, solver, board_bins, certainty_grid


def build_board(board_bins):
    return [
        [PIECE_TAG_IDS.get(tags[0].tag_id, "?") if tags else "." for tags in row]
        for row in reversed(board_bins[:8])
    ]


def print_board_array(board):
    print("\n".join(" ".join(row) for row in board))


def prompt_for_move(game_state, cam, detector):
    while True:
        input("Press Enter after move has been made...")
        img, solver, board_bins, certainty_grid = capture_board_state(cam, detector)
        if img is None:
            continue

        board_overlay_plot(img, solver, board_bins, certainty_grid)
        board = build_board(board_bins)
        game_state.print_board()
        print_board_array(board)

        move = GameState.get_move_diff(game_state, board)
        if move:
            return move, board
        print("No move detected. Adjust board and try again.")


def verify_move(expected_move, game_state, cam, detector):
    while True:
        move_check, scanned_board = prompt_for_move(game_state, cam, detector)
        if move_check == expected_move:
            print("Engine move verified.")
            return scanned_board
        print(f"Mismatch. Expected {expected_move} ({game_state.get_algebraic(expected_move)}), "
              f"but detected {move_check} ({game_state.get_algebraic(move_check)}). Try again.")


def is_capture(move: str, board: chess.Board) -> bool:
    """Check if the move captures an opponent's piece"""
    from_square = chess.parse_square(move[0:2])
    to_square = chess.parse_square(move[2:4])

    # Check if there's a piece at the destination square
    return board.piece_at(to_square) is not None


def robot_thread_function(robot: ChessMovementController):
    """Thread that handles robot movement"""
    global ai_move, move_executed, robot_running
    rospy.loginfo(
        f"ai_move: {ai_move}, move_executed: {move_executed}, robot_running: {robot_running}")
    while robot_running:
        current_move = ai_move
        if current_move:
            try:
                move_uci, is_capture = current_move  # Unpack move and capture info
                rospy.loginfo(f"Robot executing move: {move_uci} (is_capture: {is_capture})")
                success = robot.execute_move(move_uci, is_capture)
                if success:
                    rospy.loginfo(f"Robot successfully executed move: {move_uci}")
                else:
                    rospy.logerr(f"Robot failed to execute move: {move_uci}")
            except Exception as e:
                rospy.logerr(f"Error executing move: {str(e)}")
            finally:
                # Clear the move and signal completion
                ai_move = None
                move_executed.set()
        time.sleep(0.1)

    rospy.loginfo("Robot thread shutting down")


def set_robot_move(move, is_captured: bool):
    """Set the move for the robot to execute and wait for completion"""
    global ai_move, move_executed

    # Signal the robot thread to execute the move
    ai_move = (move, is_captured)
    move_executed.clear()

    # Wait for the robot to complete the move
    if not move_executed.wait(timeout=100):
        rospy.logerr("Robot movement timed out")
        return False
    return True


def main():
    global move_executed, ai_move, robot_running

    rospy.loginfo("Starting Chess Robot System...")

    test_mode = (len(sys.argv) > 1 and sys.argv[1] == 'test')
    simulation_mode = rospy.get_param('sim', True)
    rospy.loginfo(f"Command-line args: {sys.argv}")
    rospy.loginfo(f"Test mode: {test_mode}, Simulation mode: {simulation_mode}")

    # Get user preference for engine color
    engine_is_white = input("Should the engine play as White? (y/n): ").strip().lower() == 'y'

    # Initialize chess engine (for both test and normal mode)
    engine = Engine(
        engine_path=ENGINE_PATH,
        search_depth=12,
        force_elo=1500,
        opening_book_path=OPENING_BOOK_PATH
    )
    game = GameState(engine, engine_plays_white=engine_is_white)

    skeleton: Skeleton = Skeleton(RobotUR10eGripper(is_gripper_up=True))
    orchestrator: Orchestrator = Orchestrator(
        skeleton=skeleton,
        require_viz=True,
        require_approval=True
    )

    # Initialize robot movement controller - simulation mode is independent of test mode
    robot = ChessMovementController(
        orchestrator=orchestrator,
        simulation_mode=simulation_mode,
        robot_is_white=engine_is_white
    )

    # Start robot movement thread
    robot_running = True
    robot_thread = threading.Thread(target=robot_thread_function, args=(robot,))
    robot_thread.daemon = True
    robot_thread.start()

    try:
        # OPTION 1: TEST MODE - simple chess move testing without vision
        if test_mode:
            rospy.loginfo(f"Running in TEST mode (sim={simulation_mode})")

            # !!!! TODO: temporarily comment for faster testing
            # robot.test_board_calibration()
            # !!!!

            # Engine plays first if it's white
            if engine_is_white:
                move = game.get_engine_move()
                print(f"\nEngine plays first as White: {move} ({game.get_algebraic(move)})")
                rospy.loginfo(f"Robot executing engine's move: {move}")
                before = game.board.copy()
                is_captured = is_capture(move, before)
                success = set_robot_move(move, is_captured)
                if success:
                    # Update the game state
                    game.offer_move(move, by_white=True)
                    game.print_board()
                    print(f"FEN: {game.get_fen()}")
                    print("Stockfish:", engine.get_eval_score())
                    print("-" * 60)

            rospy.loginfo("Enter 'quit' to exit test mode.")
            # Test mode game loop
            while not game.board.is_game_over() and not rospy.is_shutdown():
                try:
                    human_move = input("Enter your move (e2e4) or 'quit': ").strip().lower()
                    if human_move == 'quit':
                        break

                    if len(human_move) != 4:
                        print("Invalid move format. Please use format like 'e2e4'")
                        continue

                    before = game.board.copy()
                    if game.offer_move(human_move):
                        rospy.loginfo(f"Robot executing human move: {human_move}")
                        is_captured = is_capture(human_move, before)
                        if set_robot_move(human_move, is_captured):
                            game.print_board()
                            print(f"FEN: {game.get_fen()}")
                            print("Stockfish:", engine.get_eval_score())
                            print("-" * 60)
                        else:
                            game.board = before
                            rospy.logerr("Robot failed to execute human's move")
                            continue
                    else:
                        print("Move rejected. Retaining previous board state.")
                        print("FEN:", before.fen())
                        print("-" * 60)
                        continue

                    # engine's reply
                    if game.board.turn == (chess.WHITE if engine_is_white else chess.BLACK):
                        engine_move = game.get_engine_move()
                        before = game.board.copy()
                        print(f"\nEngine move: {engine_move} ({game.get_algebraic(engine_move)})")
                        if game.offer_move(engine_move):
                            rospy.loginfo(f"Robot executing engine's move: {engine_move}")
                            is_captured = is_capture(engine_move, before)
                            if set_robot_move(engine_move, is_captured):
                                game.print_board()
                                print(f"FEN: {game.get_fen()}")
                                print("Stockfish:", engine.get_eval_score())
                                print("-" * 60)
                            else:
                                game.board = before
                                rospy.logerr("Robot failed to execute engine's move")
                                continue
                        else:
                            print("Move rejected. Retaining previous board state.")
                            print("FEN:", before.fen())
                            print("-" * 60)
                            continue

                except KeyboardInterrupt:
                    break
        # OPTION 2: FULL MODE (vision-based)
        else:
            rospy.loginfo(f"Running in FULL mode (sim={simulation_mode})")
            cam = WebcamSource(cam_id=0)
            detector = instantiate_detector()

            # Engine plays first if it's white
            if engine_is_white:
                move = game.get_engine_move()
                print(f"\nEngine plays first as White: {move} ({game.get_algebraic(move)})")
                before = game.board.copy()
                is_captured = is_capture(move, before)
                # Execute the move on the robot
                success = set_robot_move(move, is_captured)
                if success:
                    print("Robot completed the engine's move")

                # Verify the move was made correctly on the board
                scanned_board = verify_move(move, game, cam, detector)
                # Update the game state
                game.offer_move(move, by_white=True)
                # Short delay after move completes
                time.sleep(2)
                print("FEN:", game.get_fen())
                print("-" * 60)

            rospy.loginfo("Press Ctrl+C to exit.")

            # Main game loop
            while not game.board.is_game_over():
                # Wait for human player's move
                move, scanned_board = prompt_for_move(game, cam, detector)

                # validate the move
                board_before = game.board.copy()
                if game.offer_move(move):
                    print(f"Detected move: {move} ({board_before.san(chess.Move.from_uci(move))})")
                    print("FEN:", game.get_fen())
                    print("Stockfish:", engine.get_eval_score())
                else:
                    print("Move rejected. Retaining previous board state.")
                    print("FEN:", board_before.fen())
                    print("-" * 60)
                    continue

                # Engine's turn
                if game.board.turn == (chess.WHITE if engine_is_white else chess.BLACK):
                    move = game.get_engine_move()
                    print(f"\nEngine move: {move} ({game.get_algebraic(move)})")
                    before = game.board.copy()
                    is_captured = is_capture(move, before)
                    # Execute the move on the robot
                    success = set_robot_move(move, is_captured)
                    if success:
                        print("Robot completed the engine's move")
                    else:
                        print("Robot failed to execute move. Please try again.")
                        continue
                    # Verify the move was made correctly on the board
                    scanned_board = verify_move(move, game, cam, detector)
                    # Update the game state
                    game.offer_move(move, by_white=game.engine_plays_white)
                    print("FEN:", game.get_fen())
                    print("Stockfish:", engine.get_eval_score())
                    print("-" * 60)

            print("Game over.")

    except KeyboardInterrupt:
        print("\nGame interrupted by user.")
    except Exception as e:
        rospy.logerr(f"Error in game: {str(e)}")
        print(f"Error: {e}")
    finally:
        # Clean up resources
        robot_running = False
        if robot_thread.is_alive():
            robot_thread.join(timeout=2)
        if 'cam' in locals():
            cam.release()
        print("Resources cleaned up.")


if __name__ == "__main__":
    main()
