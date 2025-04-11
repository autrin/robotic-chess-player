import chess
from stockfish import Stockfish


class Engine:
    def __init__(self, engine_path, depth=20, level=15):
        self.stockfish = Stockfish(engine_path)
        self.stockfish.set_depth(depth)
        self.stockfish.set_skill_level(level)
        self.board = chess.Board()

    def set_fen(self, fen):
        self.board.set_fen(fen)
        self.stockfish.set_fen_position(fen)

    def get_best_move(self):
        move = self.stockfish.get_best_move()
        self.board.push_uci(move)
        self.stockfish.set_fen_position(self.board.fen())
        return move

    def make_opponent_move(self, move):
        m = chess.Move.from_uci(move)
        if m in self.board.legal_moves:
            self.board.push(m)
            self.stockfish.set_fen_position(self.board.fen())

    def get_fen(self):
        return self.board.fen()
