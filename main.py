#!/usr/bin/env python3
import rospy
from dev.GamePlay import GamePlayClass
from dev.ChessMovement import ChessMovementController
import time
import threading

class ChessRobot:
    def __init__(self):
        self.game = GamePlayClass()
        self.robot = ChessMovementController()
        self.running = True
        self.move_queue = []
        
    def vision_thread(self):
        """Run the game vision system"""
        self.game.play()
        
    def robot_thread(self):
        while self.running:
            if len(self.move_queue) > 0:
                move = self.move_queue.pop(0)
                try:
                    print(f"Robot starting to execute move: {move}")
                    self.robot.execute_move(move)
                    print(f"Robot finished executing move: {move}")
                except Exception as e:
                    rospy.logerr(f"Error executing move: {e}")
            time.sleep(0.1)
                
    def run(self):
        """Run the complete system"""
        # Start vision in a separate thread
        vision_thread = threading.Thread(target=self.vision_thread)
        vision_thread.daemon = True
        vision_thread.start()
        
        # Start robot control thread
        robot_thread = threading.Thread(target=self.robot_thread)
        robot_thread.daemon = True
        robot_thread.start()
        
        # Main loop - monitor for AI moves to execute
        try:
            while True:
                # Check if AI has made a move decision
                if self.game.turn == "ai" and hasattr(self.game, "ai_move"):
                    move = self.game.ai_move
                    if move and move not in self.move_queue:
                        self.move_queue.append(move)
                        
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            self.running = False
            rospy.signal_shutdown("User requested shutdown")
            
if __name__ == "__main__":
    try:
        chess_robot = ChessRobot()
        chess_robot.run()
    except rospy.ROSInterruptException:
        pass