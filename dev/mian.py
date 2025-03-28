#!/usr/bin/env python3
import rospy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from GamePlay import GamePlayClass
from practice.ur10e_pick_and_drop import UR10eControl
from ChessEngine import ChessEngineClass

"""
Acts as the central coordinator. This node will:
  Receive the current board state from the vision system.
  Convert the visual input into a FEN string.
  Send the FEN to the chess engine and get the best move.
  Translate the chess move into commands for the arm control system (including safe positioning, e.g., high,
  low, backup, and ensuring a safety gap between the piece and board).
  Command the gripper (Robotic Hand E) to open/close and include logic for reinitialization if it shuts off.
  Integrate an emergency stop mechanism.
"""

def main():
    rospy.init_node('main_integration_node', anonymous=True)
    # now the node is registered and we can create publisher and subscribers, etc.
    rospy.loginfo("Main integration node has been initialized.")

    # the node's logic goes here
    # Instantiate the modules:
    gameplay = GamePlayClass()
    arm_control = UR10eControl()
    chess_engine = ChessEngineClass()
    
    rospy.spin() # keeps the node from exiting until shutdown.
if __name__ == '__main__':
  main()