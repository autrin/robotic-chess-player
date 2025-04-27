#!/usr/bin/env python3
import rospy
import threading
import time
import sys
from ChessMovement import ChessMovementController
from ChessEngine import ChessEngineClass
from GamePlay import GamePlayClass

"""
 The primary ROS node entry point launched by chess_robot.launch file
"""

def main():
    try:
        rospy.loginfo("Starting Chess Robot System...")
        
        # Initialize the chess engine
        chess_engine = ChessEngineClass(
            enginePath="/usr/games/stockfish",
            depth=20,
            level=15
        )
        
        # Initialize robot movement controller
        movement_controller = ChessMovementController(simulation_mode=True, chess_engine=chess_engine)
        
        # detect test mode
        test_mode = (len(sys.argv) > 1 and sys.argv[1] == 'test')
        rospy.loginfo(f"Command-line args: {sys.argv}")
        rospy.loginfo(f"Test mode: {test_mode}")

        # OPTION 1: Simple chess move test mode
        if test_mode:
            rospy.loginfo("Running in test mode")
            
            # Test basic calibration
            movement_controller.test_board_calibration()
            
            # Wait for user to enter moves to test
            while not rospy.is_shutdown():
                test_move = input("Enter a chess move to test (e.g., e2e4), or 'q' to quit: ")
                if test_move.lower() == 'q':
                    break
                    
                if len(test_move) >= 4:
                    # Get AI move response
                    chess_engine.makeOppMove(test_move)
                    ai_move = chess_engine.makeAIMove()
                    
                    # Execute the player's move first
                    rospy.loginfo(f"Executing human move: {test_move}")
                    success = movement_controller.execute_move(test_move)
                    
                    if success:
                        # Then execute the AI's response
                        rospy.loginfo(f"Executing AI response: {ai_move}")
                        success = movement_controller.execute_move(ai_move)
        
        # OPTION 2: Full gameplay with vision system
        else:
            rospy.loginfo("Starting full gameplay with vision system")
            game = GamePlayClass()
            game.play(computerScreen=False)
        
        # Keep node running
        rospy.loginfo("Chess Robot System running. Press Ctrl+C to exit.")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Chess Robot System shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Chess Robot System: {e}")

if __name__ == '__main__':
    main()