from stockfish import Stockfish
import chess
import FilePathFinder

"""
Purpose:
    The script wraps Stockfish (a strong open-source chess engine) with additional functionality that:
        Maintains an internal board representation.
        Converts between standard chess notation (like FEN and algebraic coordinates) and its internal structure.
        Executes moves on the internal board.
        Retrieves the engine's best move.

Key Components:
    Initialization: Set engine parameters, board state, and other game-related variables.
    Board Representation: Convert the board state from a FEN string into a manipulable internal structure.
    Move Execution: Apply moves (including simple checks for castling) and update the board.
    FEN Generation: Rebuild the board part of the FEN string from the internal state.
    Stockfish Interaction: Query Stockfish for the best move given the current position.
"""

class ChessEngineClass:

    def __init__(
        self,
        depth=20,
        level=15,
        side="w",
        enginePath=FilePathFinder.getPath("chessEngine"),
    ):
        """
        Parameters:
        - depth: The search depth for Stockfish; the code ensures it's at least 20.
        - level: The skill level for Stockfish (also forced to be at least 20).
        - side: Which side the engine plays, 'w' for white or 'b' for black.
        - startFirst: Boolean indicating whether the engine moves first.
        - enginePath: The file path to the Stockfish executable; retrieved from a helper module.
        """

        self.depth = max(20, depth)
        self.level = max(20, level)
        self.side = side
        self.stockfish = Stockfish(enginePath)
        self.stockfish.set_depth(self.depth)
        self.stockfish.set_skill_level(self.level)
        self.FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"#"3q2k1/8/8/8/8/8/8/1Q2K3 w - - 0 1" #<--for demo2 #"rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"  # for other purposes. Might me removed
        self.board = chess.Board(self.FEN)
        self.chess = chess

    def getFromTo(self, move):
        return move[:2], move[2:]
    
    def makeAIMove(self):
        self.stockfish.set_fen_position(self.FEN)
        move = self.stockfish.get_best_move()
        self.board.push_uci(move)
        self.FEN = self.board.fen()
        return move
    
    #assume that the opponent will make the right movements
    def makeOppMove(self,move):
        move = chess.Move.from_uci(move)
        if move in self.board.legal_moves: #for demo2
            self.board.push(move)
            self.FEN = self.board.fen()
            #print(self.FEN)
    
        

    
    
    
    
        

if __name__ == "__main__":
    c = ChessEngineClass()
    print(c.FEN)
    print(c.makeAIMove())
    print(c.FEN)
    print(c.makeAIMove())
    print(c.FEN)