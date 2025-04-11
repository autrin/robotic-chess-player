import random
from typing import Optional

import chess
from stockfish import Stockfish

from jh1.typealias import List2D


class Engine:
    """
    Wrapper for the Stockfish chess engine integrated with python-chess for board state tracking.
    """


    def __init__(
        self,
        engine_path: str,
        opening_book_path: str,
        engine_plays_white: bool,
        *,
        search_depth: int = 12,
        force_elo: int = 1500
    ) -> None:
        """
        Initialize the engine with Stockfish binary path, search depth, and skill level.

        :param engine_path: Path to the Stockfish engine binary.
        :param search_depth: How many plies ahead will Stockfish search for.
        :param force_elo: Sets the elo rating of Stockfish
        """
        self.stockfish: Stockfish = Stockfish(engine_path)
        self.stockfish.set_depth(search_depth)
        self.stockfish.set_elo_rating(force_elo)
        self.board: chess.Board = chess.Board()
        self.book_path: str = opening_book_path
        self.engine_plays_white: bool = engine_plays_white


    def set_fen(self, fen: str) -> None:
        """
        Set the current position using a FEN string.

        :param fen: FEN string representing the board state.
        """
        self.board.set_fen(fen)
        self.stockfish.set_fen_position(fen)


    def make_engine_move(self) -> str:
        """
        Make a move if it's the engine's turn. Raises otherwise.

        :return: Move in UCI format.
        """
        if self.board.turn != (chess.WHITE if self.engine_plays_white else chess.BLACK):
            raise RuntimeError("[jh1.core.Engine.make_engine_move()]: Not engine's turn")

        book_move = self._get_book_move()
        if book_move:
            return book_move

        move: str = self.stockfish.get_best_move()
        self.board.push_uci(move)
        self.stockfish.set_fen_position(self.board.fen())
        return move


    def _get_book_move(self) -> Optional[str]:
        """
        Get a move from the opening book, if available.

        :return: A move in UCI format, or None if no book move found.
        """

        if not self.book_path:
            return None
        try:
            with chess.polyglot.open_reader(self.book_path) as reader:
                entries = list(reader.find_all(self.board))
                if not entries:
                    return None
                move = random.choice(entries).move()
                self.board.push(move)
                self.stockfish.set_fen_position(self.board.fen())
                return move.uci()
        except Exception:
            return None


    def get_fen(self) -> str:
        """
        Get the current board state in FEN format.

        :return: FEN string of current board position.
        """
        return self.board.fen()


    def set_engine_side(self, is_white: bool) -> None:
        self.engine_plays_white = is_white


    def is_white_turn(self) -> bool:
        """
        Return True if it's White's turn to move.
        """
        return self.board.turn == chess.WHITE


    def get_board_array(self) -> List2D[str]:
        """
        Return the current board as a 2D array of characters.
        Empty squares are represented as '.'.

        :return: 8x8 list of piece symbols or '.' for empty squares.
        """
        board_array: List2D[str] = [['.' for _ in range(8)] for _ in range(8)]
        for square in chess.SQUARES:
            piece: chess.Piece = self.board.piece_at(square)
            if piece:
                row = 7 - (square // 8)
                col = square % 8
                board_array[row][col] = piece.symbol()
        return board_array


    def print_board(self) -> None:
        """
        Print the current board in ASCII format.
        """
        print(self.board)


    def offer_move(self, move: str, by_white: Optional[bool] = None) -> bool:
        """
        Attempt to make a move. Returns False if the move is illegal or out of turn.

        :param move: UCI move string (e.g., 'e2e4').
        :param by_white: If specified, checks that the move is made by the correct side.
        :return: True if move is legal and accepted, False otherwise.
        """
        m: chess.Move = chess.Move.from_uci(move)

        if m not in self.board.legal_moves:
            print(f"[jh1.core.Engine.offer_move()]: Move {m} is not legal")
            return False

        if by_white is not None and by_white != self.is_white_turn():
            print(f"[jh1.core.Engine.offer_move()]: Move {m} is out of turn")
            return False

        self.board.push(m)
        self.stockfish.set_fen_position(self.board.fen())
        return True
