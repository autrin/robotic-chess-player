#!/usr/bin/env python3
import rospy
import math
from RobotInterface import RobotArmController
from GripperInterface import GripperController

class ChessMovementController:
    def __init__(self):
        # Initialize robot arm and gripper
        self.arm = RobotArmController()
        self.gripper = GripperController()
        
        # Board configuration (will need calibration)
        self.board_origin = [0.4, 0.3, 0.1]  # Bottom-left corner coordinates (x, y, z)
        self.square_size = 0.05              #TODO Square size in meters (adjust based on actual board)
        self.hover_height = 0.1              #TODO Height above board for safety movements
        self.piece_height = 0.05             #TODO Height of chess pieces
        
        # Initialize positions
        self.arm.move_to_position("home")
        self.gripper.activate()
        
    def square_to_position(self, square):
        """Convert chess square (e.g., 'e4') to physical position"""
        col = ord(square[0]) - ord('a')
        row = int(square[1]) - 1
        
        # Calculate position (adjust based on board orientation)
        # In this setup, a1 is at board_origin, h8 is at the far corner
        x = self.board_origin[0] + col * self.square_size
        y = self.board_origin[1] + row * self.square_size
        z = self.board_origin[2]  # Base height of the board
        
        return [x, y, z]
        
    def execute_move(self, move):
        """Execute a chess move (e.g., 'e2e4')"""
        rospy.loginfo(f"Executing move: {move}")
        
        # Parse the move
        from_square = move[0:2]
        to_square = move[2:4]
        
        # Get positions
        from_pos = self.square_to_position(from_square)
        to_pos = self.square_to_position(to_square)
        
        # Execute the move
        success = self._move_piece(from_pos, to_pos)
        
        if success:
            rospy.loginfo(f"Successfully completed move {move}")
        else:
            rospy.logerr(f"Failed to complete move {move}")
            
        return success
        
    def _move_piece(self, from_pos, to_pos):
        """Handle the actual movement sequence with safety positions"""
        try:
            # Move to high safety position
            self.arm.move_to_position("high")
            
            # Move above source position
            self._move_above(from_pos)
            
            # Lower to pick up piece
            self._move_down_to(from_pos)
            
            # Grab piece
            self.gripper.close(0.5)  # Adjust grip force as needed
            
            # Lift piece
            self._move_above(from_pos)
            
            # Move to high safety position again
            self.arm.move_to_position("high")
            
            # Move above destination
            self._move_above(to_pos)
            
            # Lower to place piece (with safety gap)
            self._move_down_to(to_pos, offset=0.005)
            
            # Release piece
            self.gripper.open()
            
            # Return to safe position
            self.arm.move_to_position("high")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error executing move: {e}")
            # Try to return to a safe position
            self.arm.move_to_position("backup")
            return False
            
    def _move_above(self, pos):
        """Move to a position above the given position"""
        return self.arm.move_to_xyz(pos[0], pos[1], pos[2] + self.hover_height)
        
    def _move_down_to(self, pos, offset=0):
        """Move down to the given position with optional small offset"""
        piece_top = pos[2] + self.piece_height + offset
        return self.arm.move_to_xyz(pos[0], pos[1], piece_top)