#! /usr/bin/env python3
"""
A standalone vision-based chess system
"""
import os

import rospy

from jh1.core import GameManager
from _environment import setup_environment

# Configure engine and resource paths
resources_dir = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "resources"
)

# sudo apt install stockfish
ENGINE_PATH = "/usr/games/stockfish"

# Opening book contains various chess openings from masters' games
OPENING_BOOK_PATH = os.path.join(resources_dir, "baron30.bin")

# Configure engine difficulty
ENGINE_ELO = 800
ENGINE_SEARCH_DEPTH = 20

# Verification features
REQUIRE_VERIFY_ENGINE_MOVE = False
REQUIRE_MOVE_APPROVAL = False
REQUIRE_MOVE_VISUALIZATION = False

if __name__ == '__main__':
    rospy.loginfo("Starting Chess Robot System...")
    manager = GameManager(
        engine_path=ENGINE_PATH,
        opening_book_path=OPENING_BOOK_PATH,
        engine_search_depth=ENGINE_SEARCH_DEPTH,
        engine_elo=ENGINE_ELO,
        require_verify_engine_move=REQUIRE_VERIFY_ENGINE_MOVE,
        require_move_approval=REQUIRE_MOVE_APPROVAL,
        require_move_viz=REQUIRE_MOVE_VISUALIZATION
    )
    setup_environment(cleanup_callback=manager.cleanup)
    try:
        manager.run()
    except KeyboardInterrupt:
        print("\nGame interrupted by user.")
    finally:
        manager.cleanup()
