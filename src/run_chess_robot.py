#!/usr/bin/env python3
import rospy
import threading
import time
from GripperInterface import GripperController
from RobotInterface import RobotArmController
from ChessMovement import ChessMovementController
from ChessEngine import ChessEngineClass

def main():
    try:
        # Remove the rospy.init_node call since roslaunch has already initialized the node
        # rospy.init_node('chess_robot_system', anonymous=True)
        
        # Initialize the chess engine
        chess_engine = ChessEngineClass(
            enginePath="/usr/games/stockfish"  # Make sure this path is correct
        )
        
        # Initialize robot movement controller
        chess_movement = ChessMovementController()
        
        # Test basic movements
        print("Moving to home position")
        chess_movement.arm.move_to_position("home")
        
        print("Opening gripper")
        chess_movement.gripper.open()
        
        print("Closing gripper")
        chess_movement.gripper.close(0.5)
        
        print("Moving to backup position")
        chess_movement.arm.move_to_position("backup")
        
        # Test a chess move
        test_move = input("Enter a chess move to test (e.g., e2e4): ")
        if test_move:
            chess_movement.execute_move(test_move)
        
        # Keep node running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()