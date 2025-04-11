from random import random
from typing import Optional

import chess
from stockfish import Stockfish


class Engine:
    """
    Wrapper for the Stockfish chess engine.
    """

    def __init__(self, engine_path: str, *, search_depth: int = 12, force_elo: int = 1500,
                 opening_book_path: Optional[str] = None) -> None:
        """
        Initialize the engine with Stockfish binary path, search depth, and skill level.

        :param engine_path: Path to the Stockfish engine binary.
        :param search_depth: How many plies ahead will Stockfish search for.
        :param force_elo: Sets the elo rating of Stockfish
        :param opening_book_path: Path to the Polyglot opening book file.
        """
        self.stockfish: Stockfish = Stockfish(engine_path)
        self.stockfish.set_depth(search_depth)
        self.stockfish.set_elo_rating(force_elo)
        self.book_path = opening_book_path

    def set_position(self, fen: str) -> None:
        """
        Set the current position using a FEN string.

        :param fen: FEN string representing the board state.
        """
        self.stockfish.set_fen_position(fen)

    def get_best_move(self) -> str:
        """
        Get the best move from the engine.

        :return: Move in UCI format.
        """
        return self.stockfish.get_best_move()

    def get_book_move(self, board: chess.Board) -> Optional[str]:
        """
        Get a move from the opening book, if available.

        :param board: Current board state.
        :return: A move in UCI format, or None if no book move found.
        """
        if not self.book_path:
            return None
        try:
            with chess.polyglot.open_reader(self.book_path) as reader:
                entries = list(reader.find_all(board))
                if not entries:
                    return None
                return random.choice(entries).move().uci()
        except Exception:
            return None

    def get_eval_score(self) -> Optional[float]:
        """
        Get the current evaluation score from the engine.

        :return: Evaluation in pawns (positive for White, negative for Black), or None if unavailable.
        """
        info = self.stockfish.get_evaluation()
        if info["type"] == "cp":
            return info["value"] / 100.0
        elif info["type"] == "mate":
            sign = 1 if info["value"] > 0 else -1
            return sign * 100.0  # convention for mate in N
        return None
