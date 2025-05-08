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
# 500: beginner
# 1000: intermediate
# 1500: advanced
# 2000: pre-master
# 2500: masters
# 3000: superhuman
# 3500+: max power
ENGINE_ELO = 1000

# Lower = faster evaluation
ENGINE_MIN_SEARCH_DEPTH = 17
ENGINE_MAX_SEARCH_DEPTH = 27

# Verification features
REQUIRE_VERIFY_ENGINE_MOVE = False
REQUIRE_MOVE_APPROVAL = False
REQUIRE_MOVE_VISUALIZATION = True
REQUIRE_BOARD_TAG_VISUALIZATION = False

if __name__ == '__main__':
    rospy.loginfo("Starting Chess Robot System...")
    manager = GameManager(
        engine_path=ENGINE_PATH,
        opening_book_path=OPENING_BOOK_PATH,
        engine_min_depth=ENGINE_MIN_SEARCH_DEPTH,
        engine_max_depth=ENGINE_MAX_SEARCH_DEPTH,
        engine_elo=ENGINE_ELO,
        require_verify_engine_move=REQUIRE_VERIFY_ENGINE_MOVE,
        require_move_approval=REQUIRE_MOVE_APPROVAL,
        require_move_viz=REQUIRE_MOVE_VISUALIZATION,
        require_board_tag_viz=REQUIRE_BOARD_TAG_VISUALIZATION
    )
    setup_environment(cleanup_callback=manager.cleanup)
    try:
        manager.run()
    except KeyboardInterrupt:
        print("\nGame interrupted by user.")
    finally:
        manager.cleanup()
