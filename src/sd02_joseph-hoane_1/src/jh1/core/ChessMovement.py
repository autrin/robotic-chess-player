#!/usr/bin/env python3
"""
Chess Movement Controller

This module connects the chess game logic with the UR10e robot arm,
controlling movement for executing chess moves in the physical world.
"""
import traceback
import rospy
import time
from typing import List, Optional
from geometry_msgs.msg import Pose

from jh1.core import Orchestrator
from jh1.robotics.robot_ur10e_gripper import RobotUR10eGripper
from jh1.topology import *


class ChessMovementController:
    """
    Controls the UR10e robot and gripper to execute chess moves.
    
    This class translates chess board coordinates to physical robot 
    positions and handles the pick-and-place sequences needed to move pieces.
    """
    GRIPPER_OPEN = 0.9  # Value when gripper is fully open
    GRIPPER_CLOSED = 0.5  # TODO Default value when gripper is holding a piece

    def __init__(self, orchestrator: Orchestrator, simulation_mode=True, robot_is_white=None):
        """
        Initialize the chess movement controller.
        
        Args:
            simulation_mode: Whether to run in simulation mode or with a real robot
        """
        rospy.loginfo("Initializing ChessMovementController...")

        # Pass in an orchestrator (wrapper around IK and robot movement functions)
        self.orchestrator: Orchestrator = orchestrator

        # Initialize ROS node if not already done
        # if not rospy.core.is_initialized():
        #     rospy.init_node('chess_movement_controller', anonymous=True)

        # Initialize the robot control interface
        self.robot: RobotUR10eGripper = orchestrator.skeleton.robot
        self.simulation_mode = simulation_mode

        # TODO
        """
        Board configuration (will need calibration) in meters
        - board_origin:
            X = 0.4 meters (40cm): Distance forward from the robot base
            Y = 0.3 meters (30cm): Distance to the left from the robot base
            Z = 0.1 meters (10cm): Height above the robot base
            These coordinates define where the corner of the chess board (specifically the a1 square)
            is located in the robot's workspace.
        """
        self.board_origin = [0.4, 0.3, 0.1]  # Bottom-left corner coordinates (x, y, z)
        self.square_size = 0.05715  # Square size in meters. 2 inches (50.8mm) per square
        self.hover_height = STANDARD_UP_HEIGHT  # Height above board for safety movements
        self.piece_height = 0.0254  # Height of chess pieces
        self.approach_height = 0.05  # Height from which to approach a piece. 2 inches - more clearance for safe approach

        # Pre-defined joint positions
        """
        - home:
        Purpose: Safe initial/default position
        Description: Robot arm fully upright with the gripper pointing downward
        Joint Values:
        First joint (base): 0 (centered)
        Second joint (shoulder): -1.57 (90 degrees back)
        Third joint (elbow): 0 (straight)
        Fourth joint (wrist 1): -1.57 (90 degrees down)
        Fifth joint (wrist 2): 0 (straight)
        Sixth joint (wrist 3): 0 (straight)
        Gripper: 0 (open)
        When Used: At startup, shutdown, or when recovering from errors

        - observe:
        Purpose: Position to get a good view of the entire board
        Description: Arm slightly lowered and extended to see the chess board from above
        Joint Values:
        Less extreme angles than home position
        Elbow joint bent at 0.5 radians
        Wrist positioned to look down at the board
        When Used:
        Between moves to assess the board
        Before and after executing a move
        As an intermediate position for safety

        - prepare:
        Purpose: Ready position before making precise movements
        Description: Similar to observe but slightly rotated and positioned for approach
        Joint Values:
        Slight base rotation (0.2)
        More bent elbow (0.7)
        Adjusted wrist angle (-1.2)
        When Used: As an intermediate position before reaching for specific squares

        - retreat:
        Purpose: Position away from the board when not actively moving pieces
        Description: Arm rotated and raised higher than the observe position
        Joint Values:
        Significant base rotation (0.5)
        Higher shoulder position (-0.8)
        More bent elbow (1.0)
        Wrist angled further (-1.5)
        When Used:
        When waiting for player's move
        When giving the player more space to interact with the board
        Between games or during pause
        """
        self.positions = {  # TODO
            "home": [2.2015607992755335, -1.7744752369322718, 1.1870899200439453,
                     -2.0474611721434535, -1.5897491613971155, 2.020841360092163,
                     self.GRIPPER_OPEN],  # Home position (gripper open)
            "observe": [1.5139759222613733, -1.1724217695048829, 1.270115613937378,
                        -1.9291945896544398, -1.569782559071676, 2.0213046073913574,
                        self.GRIPPER_OPEN],
            # Position to observe the board. Adjust to see the entire large board
            "prepare": [0.2, -1.0, 0.7, -1.2, -1.57, 0, self.GRIPPER_OPEN],  # Preparation position
            "retreat": [0.5, -0.8, 1.0, -1.5, -1.57, 0, self.GRIPPER_OPEN]
            # Position away from the board
        }

        # TODO Define two storage areas for captured pieces
        self.captured_storage = {
            'white': {'x': 0.6, 'y': 0.2, 'z': 0.1, 'next_idx': 0},  # Right side of board
            'black': {'x': 0.6, 'y': 0.4, 'z': 0.1, 'next_idx': 0}  # Left side of board
        }

        # Keep an internal representation of the board to determine if a square has a piece
        self.board_state = [[None for _ in range(8)] for _ in range(8)]
        self.robot_is_white = robot_is_white
        # Initialize robot position
        self._go_to_safe_position()
        self.orchestrator.skeleton.configuration_vector = HOME_WAYPOINT.jv
        rospy.loginfo("ChessMovementController initialized and ready")

    def _go_to_safe_position(self):
        """Move the robot to a safe starting position."""
        rospy.loginfo("Moving to home position...")
        self.robot.command_robot(self.positions["home"], 4.0)  # ! might be better to be faster here
        time.sleep(0.5)  # Short delay to ensure movement completes

    def square_to_position(self, square: str) -> List[float]:
        """
        Convert chess square (e.g., 'e4') to physical position.
        
        Args:
            square: Chess notation for the square (e.g. 'e4')
            
        Returns:
            List of [x, y, z] coordinates for the center of the square
        """
        if len(square) != 2 or not ('a' <= square[0] <= 'h') or not ('1' <= square[1] <= '8'):
            rospy.logerr(f"Invalid chess square: {square}")
            raise ValueError(f"Invalid chess square: {square}")

        return WAYPOINT_TABLE[square].pos.tolist()

    def joint_angles_for_position(self, position, gripper_open=True):
        """
        Simple cartesian to joint angle conversion.
        Args:
            position: [x, y, z] position in workspace
            gripper_open: Whether the gripper should be open (True) or closed (False)
            
        Returns:
            List of 7 joint angles (6 arm joints + gripper)
        """
        # Simple approximation for joint angles based on position
        # joint_values = [
        #     -0.5 + 0.8 * (position[0] - self.board_origin[0]),  # Pan joint - adjust based on X
        #     -1.2 - 0.8 * (position[1] - self.board_origin[1]),  # Shoulder - adjust based on Y
        #     0.7 + 0.3 * (position[2] - self.board_origin[2]),  # Elbow - adjust based on Z
        #     -1.1,  # Wrist 1
        #     -1.57,  # Wrist 2
        #     0.0,  # Wrist 3
        #     self.GRIPPER_OPEN if gripper_open else self.GRIPPER_CLOSED
        # ]
        joint_values = self.orchestrator.skeleton.partial_inverse_kinematics(position).as_command()
        joint_values.append(self.GRIPPER_OPEN if gripper_open else self.GRIPPER_CLOSED)
        return joint_values

    def execute_move(
        self,
        move: str,
        is_capture: Optional[bool] = None,
        is_en_passant: Optional[bool] = None
    ) -> bool:
        """
        Execute a chess move (e.g., 'e2e4').

        Args:
            move: Chess move in algebraic notation (e.g. 'e2e4')
            is_capture: Whether a move was a capture
            is_en_passant: Whether a move was an en-passant capture
        Returns:
            True if move was executed successfully, False otherwise
        """
        rospy.loginfo(f"Executing move: {move} (capture={is_capture} en_passant={is_en_passant})")
        if len(move) < 4:
            rospy.logerr(f"Invalid move format: {move}")
            return False

        start, end = move[:2], move[2:4]

        try:
            if start == 'e1' and end in ('g1', 'c1'):
                return self.orchestrator.castling_sequence(True, (end == 'c1'))
            if start == 'e8' and end in ('g8', 'c8'):
                return self.orchestrator.castling_sequence(False, (end == 'c8'))

            if is_en_passant:
                return self.orchestrator.en_passant_sequence(start, end)

            if is_capture:
                return self.orchestrator.capture_sequence(start, end)
            else:
                return self.orchestrator.free_movement_sequence(start, end)

        except Exception as e:
            rospy.logerr(f"Error executing move: {e}")
            self._go_to_safe_position()
            return False

    def _move_piece(self, from_pos: List[float], to_pos: List[float],
                    is_capture: bool = False) -> bool:
        """
        Handle the actual movement sequence with safety positions.
        
        Args:
            from_pos: [x, y, z] source position
            to_pos: [x, y, z] destination position
            is_capture: Whether this move is a capture
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # If it's a capture and we want to remove the captured piece first
            if is_capture:
                # Move to position above destination
                above_dest = [to_pos[0], to_pos[1], to_pos[2] + self.hover_height]
                self._move_to_cartesian(above_dest, gripper_open=True)

                # Lower to pick up the captured piece
                self._move_to_cartesian(to_pos, gripper_open=True)

                # Close gripper to grab the captured piece
                self._close_gripper()

                # Lift the captured piece
                self._move_to_cartesian(above_dest, gripper_open=False)

                # Move captured piece to the side of the board (could be a predefined position)
                # captured_storage = [0.6, 0.3, 0.2]  # TODO Location where captured pieces are stored
                # Get appropriate storage position based on piece color
                # In a real implementation, you'd get the piece color from your chess engine
                piece_color = 'white' if self.robot_is_white == 'y' else 'black'
                # or
                # piece_color = 'black'  # Since the robot is white in most setups, captured pieces are black. Won't work for test mode
                captured_storage = self._move_captured_piece_to_storage(piece_color)
                self._move_to_cartesian(captured_storage, gripper_open=False)

                # Release the captured piece
                self._open_gripper()

                # Move up from captured storage
                above_storage = [captured_storage[0], captured_storage[1],
                                 captured_storage[2] + 0.1]
                self._move_to_cartesian(above_storage, gripper_open=True)

            # Step 1: Move to observation position
            self.robot.command_robot(self.positions["observe"], 5.0)

            # Step 2: Move above source position
            above_source = [from_pos[0], from_pos[1], from_pos[2] + self.hover_height]
            self._move_to_cartesian(above_source, gripper_open=True)

            # Step 3: Lower to pick up piece
            self._move_to_cartesian(from_pos, gripper_open=True)

            # Step 4: Grab piece
            self._close_gripper()

            # Step 5: Lift piece
            self._move_to_cartesian(above_source, gripper_open=False)

            # Step 6: Move to position above destination
            above_dest = [to_pos[0], to_pos[1], to_pos[2] + self.hover_height]
            self._move_to_cartesian(above_dest, gripper_open=False)

            # Step 7: Lower to place piece
            self._move_to_cartesian(to_pos, gripper_open=False)

            # Step 8: Release piece
            self._open_gripper()

            # Step 9: Move up
            self._move_to_cartesian(above_dest, gripper_open=True)

            # Step 10: Return to observation position
            self.robot.command_robot(self.positions["observe"], 5.0)

            return True

        except Exception as e:
            rospy.logerr(f"Error in _move_piece: {e}")
            # Try to return to a safe position
            self._go_to_safe_position()
            return False

    def _move_captured_piece_to_storage(self, piece_color):
        """Move a captured piece to the appropriate storage area."""
        storage = self.captured_storage['white' if piece_color == 'black' else 'black']

        # Calculate position with offset to avoid piece collisions
        offset_x = (storage['next_idx'] % 4) * 0.03  # 3cm spacing in X
        offset_y = (storage['next_idx'] // 4) * 0.03  # 3cm spacing in Y

        storage_pos = [
            storage['x'] + offset_x,
            storage['y'] + offset_y,
            storage['z']
        ]

        # Increment the index for next captured piece
        storage['next_idx'] += 1

        return storage_pos

    def _execute_castling(self, color: str, side: str) -> bool:
        """
        Execute a castling move, which involves moving both king and rook.
        
        Args:
            color: 'white' or 'black'
            side: 'kingside' or 'queenside'
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Define the squares for the king and rook moves
            if color == 'white':
                if side == 'kingside':
                    king_from = 'e1'
                    king_to = 'g1'
                    rook_from = 'h1'
                    rook_to = 'f1'
                else:  # queenside
                    king_from = 'e1'
                    king_to = 'c1'
                    rook_from = 'a1'
                    rook_to = 'd1'
            else:  # black
                if side == 'kingside':
                    king_from = 'e8'
                    king_to = 'g8'
                    rook_from = 'h8'
                    rook_to = 'f8'
                else:  # queenside
                    king_from = 'e8'
                    king_to = 'c8'
                    rook_from = 'a8'
                    rook_to = 'd8'

            # First move the king
            king_from_pos = self.square_to_position(king_from)
            king_to_pos = self.square_to_position(king_to)
            success = self._move_piece(king_from_pos, king_to_pos)
            if not success:
                return False

            # Then move the rook
            rook_from_pos = self.square_to_position(rook_from)
            rook_to_pos = self.square_to_position(rook_to)
            success = self._move_piece(rook_from_pos, rook_to_pos)

            return success

        except Exception as e:
            rospy.logerr(f"Error executing castling: {e}")
            self._go_to_safe_position()
            return False

    def _move_to_cartesian(self, position: List[float], gripper_open: bool = True) -> bool:
        """
        Move to a cartesian position.
        
        Args:
            position: [x, y, z] target position
            gripper_open: Whether gripper should be open or closed
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Safety check - ensure position is within workspace limits
            # TODO: you might need to adjust the numbers
            # workspace_limits = {
            #     'x': (0.2, 0.8), # min max in meters
            #     'y': (0.0, 0.6),
            #     'z': (0.05, 0.4)
            # }

            # if (position[0] < workspace_limits['x'][0] or position[0] > workspace_limits['x'][1] or
            #     position[1] < workspace_limits['y'][0] or position[1] > workspace_limits['y'][1] or
            #     position[2] < workspace_limits['z'][0] or position[2] > workspace_limits['z'][1]):
            #     rospy.logerr(f"Position {position} is outside of safe workspace limits!")
            #     return False

            # In a real implementation, use proper IK or MoveIt
            # For now, use our simplified joint angles calculation
            joint_values = self.joint_angles_for_position(position, gripper_open)

            # Execute the move
            rospy.loginfo(f"Moving to position: {position}")
            success = self.robot.command_robot(joint_values, 5.0)
            if not success:
                rospy.logwarn(f"Movement to {position} may not have completed sucessfully")
                return False

            return True
        except Exception as e:
            rospy.logerr(f"Error in _move_to_cartesian: {e}")
            return False

    def _open_gripper(self) -> bool:
        """
        Open the gripper.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get current joint position
            current_pos = self.robot.get_joint_pos()

            # Keep the same arm position, just open the gripper
            gripper_open_position = tuple(list(current_pos)[:6] + [self.GRIPPER_OPEN])

            rospy.loginfo("Opening gripper")
            self.robot.command_robot(gripper_open_position, 5.0)
            time.sleep(0.5)  # Give time for the gripper to open

            return True

        except Exception as e:
            rospy.logerr(f"Error opening gripper: {traceback.format_exc()}")
            return False

    def _close_gripper(self,
                       position=None) -> bool:  # TODO the closing is the piece size approximately
        """
        Close the gripper to grasp a piece.
        
        Args:
            position: Gripper position
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get current joint position
            current_pos = self.robot.get_joint_pos()
            rospy.loginfo(f"{current_pos=}  {position=}")

            # Use provided position or default to GRIPPER_CLOSED
            grip_position = position if position is not None else self.GRIPPER_CLOSED

            # Keep the same arm position, just close the gripper
            gripper_close_position = tuple(list(current_pos)[:6] + [grip_position])

            rospy.loginfo(f"Closing gripper to position {grip_position}")
            self.robot.command_robot(gripper_close_position,
                                     5.0)  # ! might need to make this faster
            time.sleep(0.5)  # Give time for the gripper to close

            return True

        except Exception as e:
            rospy.logerr(f"Error closing gripper: {traceback.format_exc()}")
            return False

    def test_board_calibration(self) -> bool:
        """
        Test the calibration of the board by moving to each corner.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            rospy.loginfo("Testing board calibration...")

            # Move to the preparation position
            self.robot.command_robot(self.positions["observe"], 3.0)

            # Move to each corner with a slight hover
            for square in ['a1', 'a8', 'h8', 'h1']:
                position = self.square_to_position(square)
                above_position = [position[0], position[1], position[2] + self.hover_height]

                # Move above the square
                rospy.loginfo(f"Moving to above {square}")
                self._move_to_cartesian(above_position)
                time.sleep(1.0)

            # Return to safe position
            self.robot.command_robot(self.positions["observe"], 2.0)

            rospy.loginfo("Board calibration test complete")
            return True

        except Exception as e:
            rospy.logerr(f"Error in board calibration test: {e}")
            self._go_to_safe_position()
            return False


if __name__ == "__main__":
    try:
        # Initialize node if not already done
        if not rospy.core.is_initialized():
            rospy.init_node('chess_movement_controller', anonymous=True)

        # Create the controller
        controller = ChessMovementController(simulation_mode=True)

        # Test board calibration
        controller.test_board_calibration()

        # Test a move
        controller.execute_move('e2e4')

        rospy.loginfo("Tests complete")

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")
