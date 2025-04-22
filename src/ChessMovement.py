import rospy
import numpy as np
from src.RobotInterface import RobotArmController
from src.GripperInterface import GripperController

class ChessMovementController:
    def __init__(self):
        self.arm = RobotArmController()
        self.gripper = GripperController()
        
        # Board configuration (will need calibration)
        self.board_origin = [0.4, 0.3, 0.0]  # Bottom-left corner coordinates
        self.square_size = 0.0254  # 1 inch = 2.54cm
        self.hover_height = 0.05  # Height above board
        self.piece_height = 0.03  # Height of pieces
        
        # Initialize arm
        self.arm.move_to_position("backup")
        self.gripper.activate()
        
    def square_to_position(self, square):
        """Convert chess square (e.g., 'e4') to physical position"""
        col = ord(square[0]) - ord('a')
        row = int(square[1]) - 1
        
        # Calculate position (adjust based on your board orientation)
        x = self.board_origin[0] + col * self.square_size
        y = self.board_origin[1] + row * self.square_size
        return [x, y]
        
    def execute_move(self, move):
        """Execute a chess move (e.g., 'e2e4')"""
        from_square = move[0:2]
        to_square = move[2:4]
        
        # Get positions
        from_pos = self.square_to_position(from_square)
        to_pos = self.square_to_position(to_square)
        
        # Move sequence with safety
        self._move_piece(from_pos, to_pos)
        
    def _move_piece(self, from_pos, to_pos):
        """Handle the actual movement sequence with safety positions"""
        # Move to high safety position
        self.arm.move_to_position("high")
        
        # Move above source position
        self._move_xyz(from_pos[0], from_pos[1], self.hover_height)
        
        # Lower to pick up piece
        self._move_xyz(from_pos[0], from_pos[1], self.piece_height)
        
        # Grab piece
        self.gripper.close(200)  # Adjust grip force as needed
        
        # Lift piece
        self._move_xyz(from_pos[0], from_pos[1], self.hover_height)
        
        # Move to high safety position again
        self.arm.move_to_position("high")
        
        # Move above destination
        self._move_xyz(to_pos[0], to_pos[1], self.hover_height)
        
        # Lower to place piece (with safety gap)
        self._move_xyz(to_pos[0], to_pos[1], self.piece_height + 0.005)
        
        # Release piece
        self.gripper.open()
        
        # Return to safe position
        self.arm.move_to_position("high")
        
    def _move_xyz(self, x, y, z):
        """Move end effector to specific XYZ position"""
        pose_goal = self.arm.group.get_current_pose().pose
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # Keep the current orientation (pointing downward)
        self.arm.group.set_pose_target(pose_goal)
        success = self.arm.group.go(wait=True)
        
        if not success:
            rospy.logwarn(f"Failed to move to position ({x}, {y}, {z})")
            
        self.arm.group.stop()
        self.arm.group.clear_pose_targets()
        return success