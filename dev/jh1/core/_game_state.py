from typing import Optional

import chess

from ._engine import Engine
from jh1.typealias import List2D


class GameState:
    """
    Handles gameplay logic including board state and integration with the engine.
    """


    def __init__(self, engine: Engine, engine_plays_white: bool) -> None:
        """
        Initialize gameplay with the given engine.

        :param engine: Instance of the Engine class.
        :param engine_plays_white: Boolean indicating if the engine plays as White.
        """
        self.engine = engine
        self.board: chess.Board = chess.Board()
        self.engine_plays_white = engine_plays_white


    def set_fen(self, fen: str) -> None:
        """
        Set the current position using a FEN string.

        :param fen: FEN string representing the board state.
        """
        self.board.set_fen(fen)
        self.engine.set_position(fen)


    def get_fen(self) -> str:
        """
        Get the current board state in FEN format.

        :return: FEN string of current board position.
        """
        return self.board.fen()


    def make_engine_move(self) -> str:
        """
        Make a move if it's the engine's turn. Raises otherwise.

        :return: Move in UCI format.
        """
        if self.board.turn != (chess.WHITE if self.engine_plays_white else chess.BLACK):
            raise RuntimeError("[Gameplay.make_engine_move()]: Not engine's turn")

        book_move = self.engine.get_book_move(self.board)
        if book_move:
            self.board.push_uci(book_move)
            self.engine.set_position(self.board.fen())
            return book_move

        move = self.engine.get_best_move()
        self.board.push_uci(move)
        self.engine.set_position(self.board.fen())
        return move


    def offer_move(self, move: str, by_white: Optional[bool] = None) -> bool:
        """
        Attempt to make a move. Returns False if the move is illegal or out of turn.

        :param move: UCI move string (e.g., 'e2e4').
        :param by_white: If specified, checks that the move is made by the correct side.
        :return: True if move is legal and accepted, False otherwise.
        """
        m = chess.Move.from_uci(move)
        if m not in self.board.legal_moves:
            print(f"[Gameplay.offer_move()]: Move {m} is not legal")
            return False

        if by_white is not None and by_white != self.board.turn == chess.WHITE:
            print(f"[Gameplay.offer_move()]: Move {m} is out of turn")
            return False

        self.board.push(m)
        self.engine.set_position(self.board.fen())
        return True


    def is_white_turn(self) -> bool:
        """
        Return True if it's White's turn to move.
        """
        return self.board.turn == chess.WHITE


    def set_engine_side(self, is_white: bool) -> None:
        """
        Set which side the engine is playing.

        :param is_white: True if engine plays White, False if Black.
        """
        self.engine_plays_white = is_white


    def get_board_array(self) -> List2D[str]:
        """
        Return the current board as a 2D array of characters.
        Empty squares are represented as '.'.

        :return: 8x8 list of piece symbols or '.' for empty squares.
        """
        board_array: List2D[str] = [['.' for _ in range(8)] for _ in range(8)]
        for square in chess.SQUARES:
            piece = self.board.piece_at(square)
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
