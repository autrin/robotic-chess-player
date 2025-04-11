import chess
from stockfish import Stockfish

from jh1.typealias import List2D


class Engine:
    """
    Wrapper for the Stockfish chess engine integrated with python-chess for board state tracking.
    """

    def __init__(self, engine_path: str, depth: int = 20, level: int = 15) -> None:
        """
        Initialize the engine with Stockfish binary path, search depth, and skill level.

        :param engine_path: Path to the Stockfish engine binary.
        :param depth: Search depth for Stockfish.
        :param level: Skill level of Stockfish (0–20).
        """
        self.stockfish: Stockfish = Stockfish(engine_path)
        self.stockfish.set_depth(depth)
        self.stockfish.set_skill_level(level)
        self.board: chess.Board = chess.Board()

    def set_fen(self, fen: str) -> None:
        """
        Set the current position using a FEN string.

        :param fen: FEN string representing the board state.
        """
        self.board.set_fen(fen)
        self.stockfish.set_fen_position(fen)

    def get_best_move(self) -> str:
        """
        Get and apply the best move from Stockfish, updating the board.

        :return: Best move in UCI format.
        """
        move: str = self.stockfish.get_best_move()
        self.board.push_uci(move)
        self.stockfish.set_fen_position(self.board.fen())
        return move

    def get_fen(self) -> str:
        """
        Get the current board state in FEN format.

        :return: FEN string of current board position.
        """
        return self.board.fen()

    def get_board_array(self) -> List2D[str]:
        """
        Return the current board as a 2D array of characters.
        Empty squares are represented as '.'.

        :return: 8x8 list of piece symbols or '.' for empty squares.
        """
        board_array = [['.' for _ in range(8)] for _ in range(8)]
        for square in chess.SQUARES:
            piece = self.board.piece_at(square)
            if piece:
                row = 7 - (square // 8)
                col = square % 8
                board_array[row][col] = piece.symbol()
        return board_array
