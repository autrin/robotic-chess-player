import pytest
import chess
from jh1.core import GameState, Engine
from typing import Optional


# noinspection PyMissingConstructor
class MockEngine(Engine):
    def __init__(self):
        self.position = ""
        self.moves = iter(["e2e4", "d7d5", "g1f3"])
        self.book = {"rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1": "e2e4"}

    def set_position(self, fen: str) -> None:
        self.position = fen

    def get_best_move(self) -> Optional[str]:
        try:
            return next(self.moves)
        except StopIteration:
            return None

    def get_book_move(self, board: chess.Board) -> Optional[str]:
        return self.book.get(board.fen())


def test_mutate_fen():
    ng = MockEngine()
    gs = GameState(ng, engine_plays_white=True)
    fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
    gs.set_fen(fen)
    assert gs.get_fen() == fen
    assert ng.position == fen


def test_engine_book():
    ng = MockEngine()
    gs = GameState(ng, engine_plays_white=True)
    fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
    gs.set_fen(fen)
    assert gs.get_engine_move() == "e2e4"


def test_offer_illegal():
    engine = MockEngine()
    game = GameState(engine, engine_plays_white=True)

    # Legal move
    move = "e2e4"
    assert game.offer_move(move) is True
    assert game.board.fullmove_number == 1
    assert engine.position == game.get_fen()

    # Illegal move
    move = "e2e5"
    assert game.offer_move(move) is False


def test_turn_order():
    engine = MockEngine()
    gs = GameState(engine, engine_plays_white=True)

    # Correct turn (White starts)
    move = gs.make_engine_move()
    assert move == "e2e4"
    assert gs.board.fullmove_number == 1

    # Wrong turn (not engine's)
    with pytest.raises(RuntimeError):
        gs.make_engine_move()


def test_board_array():
    ng = MockEngine()
    gs = GameState(ng, engine_plays_white=True)
    move = gs.make_engine_move()
    assert move == "e2e4"

    arr = gs.get_board_array()
    expected = [
        ["r", "n", "b", "q", "k", "b", "n", "r"],
        ["p", "p", "p", "p", "p", "p", "p", "p"],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        [".", ".", ".", ".", "P", ".", ".", "."],
        [".", ".", ".", ".", ".", ".", ".", "."],
        ["P", "P", "P", "P", ".", "P", "P", "P"],
        ["R", "N", "B", "Q", "K", "B", "N", "R"],
    ]

    assert isinstance(arr, list)
    assert len(arr) == 8 and all(len(row) == 8 for row in arr)

    for i in range(8):
        for j in range(8):
            assert arr[i][j] == expected[i][j], \
                f"Mismatch at ({i},{j}): expected {expected[i][j]}, got {arr[i][j]}"


def test_algebraic():
    ng = MockEngine()
    gs = GameState(ng, engine_plays_white=True)
    move = "e2e4"
    assert gs.get_algebraic(move) == "e4"
    assert gs.get_algebraic("e1e9") == "<Illegal>"


def test_basic_diff():
    ng = MockEngine()
    gs = GameState(ng, engine_plays_white=True)
    gs.offer_move("e2e4")
    next_board = GameState._board_to_array(gs.board)
    gs.board.pop()  # revert
    diff = GameState.get_move_diff(gs, next_board)
    assert diff == "e2e4"
