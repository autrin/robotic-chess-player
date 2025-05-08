from typing import Optional, Tuple, Union

import chess
import chess.pgn

from ._engine import Engine
from jh1.typealias import List2D
from ..visual import PIECE_TAG_IDS


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

        self.pgn_tree = chess.pgn.Game()
        self.node = self.pgn_tree

    def set_fen(self, fen: str) -> None:
        """
        Set the current position using a FEN string.

        :param fen: FEN string representing the board state.
        """
        self.board.set_fen(fen)
        self.engine.set_position(fen)
        self.pgn_tree.setup(fen)
        self.node = self.pgn_tree

    def get_fen(self) -> str:
        """
        Get the current board state in FEN format.

        :return: FEN string of current board position.
        """
        return self.board.fen()

    def get_engine_move(self) -> Optional[str]:
        """
        Get the engine's move (opening book or computed), but do not play it.

        :return: Move in UCI format, or None if mate found.
        """
        book_move = self.engine.get_book_move(self.board)
        if book_move:
            return book_move

        return self.engine.get_best_move()

    def make_engine_move(self) -> str:
        """
        Make a move if it's the engine's turn. Raises otherwise.

        :return: Move in UCI format.
        """
        if self.board.turn != (chess.WHITE if self.engine_plays_white else chess.BLACK):
            raise RuntimeError("[Gameplay.make_engine_move()]: Not engine's turn")

        move = self.get_engine_move()
        if not move:
            raise RuntimeError("[Gameplay.make_engine_move()]: Engine did not return a move")

        m = chess.Move.from_uci(move)
        self.board.push(m)
        self.node = self.node.add_variation(m)
        self.engine.set_position(self.board.fen())
        return move

    def is_engine_turn(self) -> bool:
        return self.board.turn == (chess.WHITE if self.engine_plays_white else chess.BLACK)

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
        self.node = self.node.add_variation(m)
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
        return GameState._board_to_array(self.board)

    def print_board(self) -> None:
        """
        Print the current board in ASCII format.
        """
        # print(self.board)
        print()
        print(f"FEN: " + self.get_fen())
        print(GameState.prettify(self.board))

    def get_algebraic(self, uci: str) -> str:
        try:
            return self.board.san(chess.Move.from_uci(uci))
        except (AssertionError, ValueError):
            return "<Illegal>"

    def get_pgn(self) -> str:
        return str(self.pgn_tree)

    @staticmethod
    def get_move_diff(game_state, new_board_array: List2D[str]) -> Optional[str]:
        """
        Determines the move that transitions the current board state (inside game_state)
        to the new board state (new_board_array). Returns the move in UCI notation if found.
        """
        legal_moves = list(game_state.board.legal_moves)
        for move in legal_moves:
            tmp_board = game_state.board.copy()
            tmp_board.push(move)
            if GameState._board_to_array(tmp_board) == new_board_array:
                return move.uci()
        return None

    @staticmethod
    def _board_to_array(board: chess.Board) -> List2D[str]:
        board_array: List2D[str] = [['.' for _ in range(8)] for _ in range(8)]
        for square in chess.SQUARES:
            piece = board.piece_at(square)
            if piece:
                row = 7 - (square // 8)
                col = square % 8
                board_array[row][col] = piece.symbol()
        return board_array

    @staticmethod
    def prettify(board: Union[chess.Board, List2D[str]]) -> str:
        unicode_pieces = {
            'P': '♟', 'N': '♞', 'B': '♝', 'R': '♜', 'Q': '♛', 'K': '♚',
            'p': '♙', 'n': '♘', 'b': '♗', 'r': '♖', 'q': '♕', 'k': '♔',
        }

        board_array: List2D[str]

        if isinstance(board, chess.Board):
            board_array = [[' ' for _ in range(8)] for _ in range(8)]
            for square in chess.SQUARES:
                piece = board.piece_at(square)
                if piece:
                    row = 7 - (square // 8)
                    col = square % 8
                    board_array[row][col] = unicode_pieces[piece.symbol()]
        else:
            board_array = [[unicode_pieces.get(cell, " ") for cell in row] for row in board]

        top = "      ┌───" + "┬───" * 7 + "┐"
        mid = "      ├───" + "┼───" * 7 + "┤"
        bot = "      └───" + "┴───" * 7 + "┘"
        rows = ["", top]

        for i, row in enumerate(board_array):
            rank = 8 - i
            line = [f"    {rank} │"]
            for cell in row:
                line.append(f" {cell} │")
            rows.append("".join(line))
            if i != 7:
                rows.append(mid)

        rows.append(bot)
        rows.append("        a   b   c   d   e   f   g   h")
        rows.append("")
        return "\n".join(rows)

    @staticmethod
    def classify_move(move: str, board: chess.Board) -> Tuple[bool, bool]:
        m = chess.Move.from_uci(move)
        return board.is_capture(m), board.is_en_passant(m)


    @staticmethod
    def build_board(board_bins) -> List2D[str]:
        """
        Convert clusters into a 2D array of piece symbols.
        """
        return [
            [PIECE_TAG_IDS.get(cell[0].tag_id, "?") if cell else "." for cell in row]
            for row in reversed(board_bins[:8])
        ]