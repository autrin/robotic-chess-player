#!/usr/bin/env python3
"""
Chess Movement Controller

This module connects the chess game logic with the UR10e robot arm,
controlling movement for executing chess moves in the physical world.
"""

import rospy
import numpy as np
import math
import time
from typing import List, Tuple, Optional
from robot_ur10e_gripper import RobotUR10eGripper

class ChessMovementController:
    """
    Controls the UR10e robot and gripper to execute chess moves.
    
    This class translates chess board coordinates to physical robot 
    positions and handles the pick-and-place sequences needed to move pieces.
    """
    
    def __init__(self, simulation_mode=True):
        """
        Initialize the chess movement controller.
        
        Args:
            simulation_mode: Whether to run in simulation mode or with a real robot
        """
        rospy.loginfo("Initializing ChessMovementController...")
        
        # Initialize the robot control interface
        self.robot = RobotUR10eGripper(is_gripper_up=True)
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
        self.square_size = 0.0508            # Square size in meters. 2 inches (50.8mm) per square
        self.hover_height = 0.1              # Height above board for safety movements
        self.piece_height = 0.0254           # Height of chess pieces
        self.approach_height = 0.05          # Height from which to approach a piece. 2 inches - more clearance for safe approach
        # TODO
        # Pre-defined joint positions
        self.positions = {
            "home": [0, -1.57, 0, -1.57, 0, 0, 0],         # Home position (gripper open)
            "observe": [0, -1.0, 0.5, -1.0, -1.57, 0, 0],  # Position to observe the board. Adjust to see the entire large board
            "prepare": [0.2, -1.0, 0.7, -1.2, -1.57, 0, 0],# Preparation position
            "retreat": [0.5, -0.8, 1.0, -1.5, -1.57, 0, 0] # Position away from the board
        }
        
        # Initialize robot position
        self._go_to_safe_position()
        rospy.loginfo("ChessMovementController initialized and ready")
    
    def _go_to_safe_position(self):
        """Move the robot to a safe starting position."""
        rospy.loginfo("Moving to home position...")
        self.robot.command_robot(self.positions["home"], 5.0)
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
            
        col = ord(square[0]) - ord('a')
        row = int(square[1]) - 1
        
        # Calculate position (adjust based on board orientation)
        # In this setup, a1 is at board_origin, h8 is at the far corner
        x = self.board_origin[0] + col * self.square_size + (self.square_size / 2)
        y = self.board_origin[1] + row * self.square_size + (self.square_size / 2)
        z = self.board_origin[2]  # Base height of the board
        
        return [x, y, z]
        
    def joint_angles_for_position(self, position: List[float], gripper_open: bool = True) -> List[float]:
        """
        Convert cartesian position to joint angles using inverse kinematics.
        
        In a real implementation, this would use proper IK through the MoveIt interface.
        This simplified version just returns a pre-calculated pose with adjusted X/Y.
        
        Args:
            position: [x, y, z] position in workspace
            gripper_open: Whether the gripper should be open (True) or closed (False)
            
        Returns:
            List of 7 joint angles (6 arm joints + gripper)
        """
        # This is a simplified approach - in real world, use proper IK
        # The joint_values should come from MoveIt or another IK solver
        
        # Example joint values for a position above the board
        # Adjust the values based on the position
        joint_values = [
            -0.5 + 0.1 * (position[0] - self.board_origin[0]),  # Pan joint - adjust based on X
            -1.2 - 0.1 * (position[1] - self.board_origin[1]),   # Shoulder - adjust based on Y
            0.7,                                                # Elbow
            -1.1,                                               # Wrist 1
            -1.57,                                              # Wrist 2
            0.0,                                                # Wrist 3
            0.0 if gripper_open else 0.7                        # Gripper (0 = open, 0.7 = closed)
        ]
        
        return joint_values
    
    def execute_move(self, move: str) -> bool:
        """
        Execute a chess move (e.g., 'e2e4').
        
        Args:
            move: Chess move in algebraic notation (e.g. 'e2e4')
            
        Returns:
            True if move was executed successfully, False otherwise
        """
        rospy.loginfo(f"Executing move: {move}")
        
        try:
            # Parse the move
            if len(move) < 4:
                rospy.logerr(f"Invalid move format: {move}")
                return False
                
            from_square = move[0:2]
            to_square = move[2:4]
            
            # Special case for castling
            if from_square == 'e1' and to_square == 'g1':  # White kingside castling
                success = self._execute_castling('white', 'kingside')
                return success
            elif from_square == 'e1' and to_square == 'c1':  # White queenside castling
                success = self._execute_castling('white', 'queenside')
                return success
            elif from_square == 'e8' and to_square == 'g8':  # Black kingside castling
                success = self._execute_castling('black', 'kingside')
                return success
            elif from_square == 'e8' and to_square == 'c8':  # Black queenside castling
                success = self._execute_castling('black', 'queenside')
                return success
            
            # Get positions
            from_pos = self.square_to_position(from_square)
            to_pos = self.square_to_position(to_square)
            
            # Check for capture - if there's a piece at the destination
            # This would need to come from your chess engine/board state
            is_capture = False  # You would determine this from your chess engine
            
            # Execute the standard move
            success = self._move_piece(from_pos, to_pos, is_capture)
            
            if success:
                rospy.loginfo(f"Successfully completed move {move}")
            else:
                rospy.logerr(f"Failed to complete move {move}")
                
            return success
            
        except Exception as e:
            rospy.logerr(f"Error executing move: {e}")
            self._go_to_safe_position()
            return False
        
    def _move_piece(self, from_pos: List[float], to_pos: List[float], is_capture: bool = False) -> bool:
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
                captured_storage = [0.6, 0.3, 0.2]  # Location where captured pieces are stored
                self._move_to_cartesian(captured_storage, gripper_open=False)
                
                # Release the captured piece
                self._open_gripper()
                
                # Move up from captured storage
                above_storage = [captured_storage[0], captured_storage[1], captured_storage[2] + 0.1]
                self._move_to_cartesian(above_storage, gripper_open=True)
            
            # Step 1: Move to observation position
            self.robot.command_robot(self.positions["observe"], 2.0)
            
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
            self.robot.command_robot(self.positions["observe"], 2.0)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error in _move_piece: {e}")
            # Try to return to a safe position
            self._go_to_safe_position()
            return False
    
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
            # In a real implementation, use proper IK or MoveIt
            # For now, use our simplified joint angles calculation
            joint_values = self.joint_angles_for_position(position, gripper_open)
            
            # Execute the move
            rospy.loginfo(f"Moving to position: {position}")
            self.robot.command_robot(joint_values, 2.0)
            
            # Add a short delay to ensure movement completes
            # In a real implementation, you would wait for feedback that movement is complete
            time.sleep(0.5)
            
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
            gripper_open_position = current_pos[:6] + [0.0]  # 0.0 is open
            
            rospy.loginfo("Opening gripper")
            self.robot.command_robot(gripper_open_position, 1.0)
            time.sleep(0.5)  # Give time for the gripper to open
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error opening gripper: {e}")
            return False
    
    def _close_gripper(self, position=0.7) -> bool:
        """
        Close the gripper to grasp a piece.
        
        Args:
            position: Gripper position (0.0 is open, 1.0 is closed)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get current joint position
            current_pos = self.robot.get_joint_pos()
            
            # Keep the same arm position, just close the gripper
            gripper_close_position = current_pos[:6] + [position]
            
            rospy.loginfo(f"Closing gripper to position {position}")
            self.robot.command_robot(gripper_close_position, 1.0)
            time.sleep(0.5)  # Give time for the gripper to close
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error closing gripper: {e}")
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
            self.robot.command_robot(self.positions["observe"], 5.0)
            
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