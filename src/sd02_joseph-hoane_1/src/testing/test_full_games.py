from io import StringIO
from typing import List, Tuple

import chess
import chess.pgn
import pytest

from jh1.core import Engine, GameState


class EmptyEngine(Engine):
    def __init__(self):
        pass

    def set_position(self, fen: str) -> None:
        self.last_fen = fen

    def get_best_move(self):
        return None

    def get_book_move(self, board: chess.Board):
        return None


def get_uci_moves_from_pgn(pgn_text: str):
    cg = chess.pgn.read_game(StringIO(pgn_text))
    moves = [move.uci() for move in cg.mainline_moves()]
    assert len(moves) > 0
    return moves


def get_board_snapshots(pgn_text: str):
    cg = chess.pgn.read_game(StringIO(pgn_text))
    board = cg.board()
    snapshots = []
    for move in cg.mainline_moves():
        board.push(move)
        snapshots.append(GameState._board_to_array(board))
    assert len(snapshots) > 0
    return snapshots


def get_fen_snapshots(pgn_text: str) -> List[str]:
    cg = chess.pgn.read_game(StringIO(pgn_text))
    board = cg.board()
    fens = []
    for move in cg.mainline_moves():
        board.push(move)
        fens.append(board.fen())
    assert len(fens) > 0
    return fens


def get_piece_maps(pgn_text: str) -> List[dict]:
    cg = chess.pgn.read_game(StringIO(pgn_text))
    board = cg.board()
    maps = []
    for move in cg.mainline_moves():
        board.push(move)
        maps.append(board.piece_map().copy())
    assert len(maps) > 0
    return maps


PGN_CASES: List[Tuple[str, str]] = [
    (
        "Kasparov vs Deep Blue, 1996, Game 1",
        """
[Event "Garry Kasparov vs Deep Blue: Game 1: Deep Blue - Kasparov"]
[Site "https://lichess.org/study/1tjS4Wyd/pzMyazPq"]
[Result "*"]
[Variant "Standard"]
[ECO "B22"]
[Opening "Sicilian Defense: Alapin Variation, Barmen Defense, Modern Line"]
[Annotator "https://lichess.org/@/Magno2204"]
[StudyName "Garry Kasparov vs Deep Blue"]
[ChapterName "Game 1: Deep Blue - Kasparov"]
[UTCDate "2020.06.18"]
[UTCTime "23:41:16"]

1. e4 c5 2. c3 d5 3. exd5 Qxd5 4. d4 Nf6 5. Nf3 Bg4 6. Be2 e6 { [%cal Gf8d6,Gd1a4,Gb8c6] } 
7. h3 { [%cal Gg4h5] } 7... Bh5 8. O-O Nc6 9. Be3 cxd4 { [%cal Gf3d4,Gh5e2,Gd5d1,Gf1d1,Ga8d8,Gb1a3] } 
10. cxd4 Bb4 11. a3 Ba5 12. Nc3 { [%csl Gf6] } 12... Qd6 13. Nb5 Qe7 14. Ne5 
Bxe2 15. Qxe2 O-O 16. Rac1 Rac8 17. Bg5 { [%csl Gf6,Gc6][%cal Gc1c8] } 
17... Bb6 18. Bxf6 gxf6 19. Nc4 Rfd8 20. Nxb6 axb6 21. Rfd1 f5 
{ [%cal Gg1h2,Gd1d8,Gc1c8,Gd1c1,Ge2b5,Ge2h5] } 22. Qe3 Qf6 23. d5 Rxd5 24. 
Rxd5 exd5 25. b3 Kh8 26. Qxb6 Rg8 27. Qc5 { [%cal Gg8g2] } 27... d4 28. Nd6 
f4 { [%csl Gh8] } 29. Nxb7 Ne5 30. Qd5 f3 31. g3 Nd3 32. Rc7 Re8 33. Nd6 Re1+ 
34. Kh2 Nxf2 { [%csl Gf2][%cal Ge1h1] } 35. Nxf7+ { [%cal Gf6f7,Gc7f7,Ge1h1,Gd5d4,Gh8g8,Gc7c8,Gd4f2] } 
35... Kg7 36. Ng5+ Kh6 37. Rxh7+ { [%cal Gh6g6,Gd5g8,Gg6f5,Gg3g4] } *
        """
    ),
    (
        "Kasparov vs Deep Blue rematch, 1997, Game 6",
        """
[Event "Garry Kasparov vs Deep Blue: Game 6: Deep Blue - Kasparov"]
[Site "https://lichess.org/study/1tjS4Wyd/eSWarjHL"]
[Result "*"]
[Variant "Standard"]
[ECO "B17"]
[Opening "Caro-Kann Defense: Karpov Variation, Modern Variation"]
[Annotator "https://lichess.org/@/Magno2204"]
[StudyName "Garry Kasparov vs Deep Blue"]
[ChapterName "Game 6: Deep Blue - Kasparov"]
[UTCDate "2020.06.18"]
[UTCTime "23:41:16"]

1. e4 c6 2. d4 d5 3. Nc3 dxe4 4. Nxe4 Nd7 5. Ng5 Ngf6 6. Bd3 e6 7. N1f3 h6
8. Nxe6 Qe7 9. O-O fxe6 10. Bg6+ Kd8 11. Bf4 b5 12. a4 Bb7 13. Re1 Nd5 14. Bg3
Kc8 15. axb5 cxb5 16. Qd3 Bc6 17. Bf5 exf5 18. Rxe7 Bxe7 19. c4 *
        """
    ),
    (  # This one ended in checkmate
        "The Opera Game, Paul Morphy vs Duke of Brunswick and Count Isouard, 1858",
        """
[Event "Paris"]
[Site "Paris FRA"]
[Date "1858.??.??"]
[EventDate "?"]
[Round "?"]
[Result "1-0"]
[White "Paul Morphy"]
[Black "Duke Karl / Count Isouard"]
[ECO "C41"]
[WhiteElo "?"]
[BlackElo "?"]
[PlyCount "33"]
[Source "(London) Field, London, 1858.12.04, p458"]

1.e4 e5 2.Nf3 d6 3.d4 Bg4 {This is a weak move
already.--Fischer} 4.dxe5 Bxf3 5.Qxf3 dxe5 6.Bc4 Nf6 7.Qb3 Qe7
8.Nc3 c6 9.Bg5 {Black is in what's like a zugzwang position
here. He can't develop the [Queen's] knight because the pawn
is hanging, the bishop is blocked because of the
Queen.--Fischer} b5 10.Nxb5 cxb5 11.Bxb5+ Nbd7 12.O-O-O Rd8
13.Rxd7 Rxd7 14.Rd1 Qe6 15.Bxd7+ Nxd7 16.Qb8+ Nxb8 17.Rd8# 1-0
        """
    ),
    (  # This one has promotions
        "Kasparov vs. Karpov, 1993",
        """
[Event "Linares"]
[Site "Linares ESP"]
[Date "1993.03.09"]
[EventDate "1993.02.23"]
[Round "10"]
[Result "0-1"]
[White "Anatoly Karpov"]
[Black "Garry Kasparov"]
[ECO "E86"]
[WhiteElo "?"]
[BlackElo "?"]
[PlyCount "54"]

1.d4 Nf6 2.c4 g6 3.Nc3 Bg7 4.e4 d6 5.f3 O-O 6.Be3 e5 7.Nge2 c6
8.Qd2 Nbd7 9.Rd1 a6 10.dxe5 Nxe5 11.b3 b5 12.cxb5 axb5 13.Qxd6
Nfd7 14.f4 b4 15.Nb1 Ng4 16.Bd4 Bxd4 17.Qxd4 Rxa2 18.h3 c5
19.Qg1 Ngf6 20.e5 Ne4 21.h4 c4 22.Nc1 c3 23.Nxa2 c2 24.Qd4
cxd1=Q+ 25.Kxd1 Ndc5 26.Qxd8 Rxd8+ 27.Kc2 Nf2 0-1
        """
    )

]


@pytest.mark.parametrize("name,pgn_text", PGN_CASES)
def test_offer_moves_from_pgn(name, pgn_text):
    print(f"\nTest {name}, via offers api")
    moves = get_uci_moves_from_pgn(pgn_text)
    gs = GameState(EmptyEngine(), engine_plays_white=True)
    for i, move in enumerate(moves):
        assert gs.offer_move(move, by_white=(i % 2 == 0)), f"{name}: Failed on move {move}"
    assert len(gs.board.move_stack) == len(moves)
    print()


@pytest.mark.parametrize("name,pgn_text", PGN_CASES)
def test_diff_detection_from_snapshots(name, pgn_text):
    print(f"\nTest {name}, via diffs api")

    snapshots = get_board_snapshots(pgn_text)
    exp_fen = get_fen_snapshots(pgn_text)
    exp_piece_maps = get_piece_maps(pgn_text)

    gs = GameState(EmptyEngine(), engine_plays_white=True)
    observed = []

    for i, snapshot in enumerate(snapshots):
        move = GameState.get_move_diff(gs, snapshot)
        assert move is not None, f"{name}: Could not infer move {i}"
        print((f"{i // 2 + 1}.".ljust(5, ' ') if i % 2 == 0 else "") +
              f"{move.ljust(5, ' ')} "
              f"| {gs.get_algebraic(move).ljust(7, ' ')} "
              # f"from diff, "
              , end="")
        success = gs.offer_move(move)
        assert success, f"{name}: Move {move} was not accepted"
        # print("accepted!", end="")
        print("", end=("\n" if i % 2 == 1 else "\t"))

        observed.append(move)

        # Verify FEN matches expected
        assert gs.board.fen() == exp_fen[i], f"{name}: FEN mismatch after move {move}"

        # Verify piece placement matches
        assert gs.board.piece_map() == exp_piece_maps[
            i], f"{name}: Piece map mismatch after move {move}"

    # Verify full move sequence
    assert [m.uci() for m in gs.board.move_stack] == observed, f"{name}: Final move stack mismatch"
    print("\nFinal board state:")
    gs.print_board()
    print()
