from stockfish import Stockfish
import re
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

class ChessEngine:

    def __init__(
        self,
        depth=20,
        level=15,
        side="w",
        startFirst=True,
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
        self.myTurn = startFirst
        
        # A dictionary self.board is created with keys 0 to 7 (representing rows) where each key holds a 
        # list (which will store the row’s content).
        self.board = {}  # for the ease of manipulation
        for i in range(0, 8):
            self.board[i] = []

        self.stockfish = Stockfish(enginePath)
        self.stockfish.set_depth(self.depth)
        self.stockfish.set_skill_level(self.level)
        self.fenPos = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"  # for other purposes. Might me removed
        self.stockfish.set_fen_position(self.fenPos)
        self.halfMoveClock = 0  # Tracks 50-move rule
        self.enPassant = "-"  # En passant target square
        self.turnClock = 1  # Turn counter
        self.castling = "KQkq"  # Available castling rights
        self.usedCastle = False  # Track castling
        self.draw = False  # Check draw condition

    def setFenPos(self):
        self.stockfish.set_fen_position(self.fenPos)

    # needed for later on. For manipulation of the string
    def getRankStartEndPos(self, rank):
        lis = self.fenPos.split(" ")
        lis = lis[0].replace(" ", "")
        lis = lis.split("/")
        # print(lis)
        startPos = 0

        for i in range(0, rank - 1):
            startPos += len(lis[i]) + 1

        return startPos, startPos + len(lis[rank - 1]) - 1

    def getFromTo(self, move):
        return move[:2], move[2:]

    # coord is in string
    def chessCoord2PosonBoard(self, coord):

        col = coord[0]
        rank = 8 - int(coord[1])
        return self.Fencol2ColOnBoard(col), rank

    # Converts column letters (a-h) to indices (0-7).
    def Fencol2ColOnBoard(self, Fencol):
        return ord(Fencol) - 97

    # Converts the FEN string into an internal board array (self.board).
    def updateBoard(self):
        for r in range(1, 9):
            sp, ep = self.getRankStartEndPos(r)
            # print(f"{sp,ep}")
            self.board[r - 1] = []
            for c in range(sp, ep + 1):
                if self.fenPos[c].isdigit():
                    for i in range(0, int(self.fenPos[c])):
                        self.board[r - 1].append("*")  # Empty squares are *
                else:
                    self.board[r - 1].append(self.fenPos[c])

    # moves a chess piece on board
    # souce and dest must be positions on the board
    # return True if a move was successful
    # return False otherwise
    # right now, assume the opponent won't make any illegal moves
    def moveSourceToDest(self, source, dest):

        # check kingside castling
        if (
            not self.usedCastle
            and source == "e1"
            and dest == "g1"
            and self.board[0][4] == "K"
            and self.board[0][7] == "R"
        ):
            if self.board[0][5] != "*" or self.board[0][6] != "*":
                return False

        # check queenside castling
        if (
            not self.usedCastle
            and source == "e1"
            and dest == "a1"
            and self.board[0][4] == "K"
            and self.board[0][1] == "R"
        ):
            if self.board[0][3] != "*" or self.board[0][2] != "*":
                return False

        sourceCol, sourceRank = self.chessCoord2PosonBoard(source)
        destCol, destRank = self.chessCoord2PosonBoard(dest)
        self.board[destRank][destCol] = self.board[sourceRank][sourceCol]
        self.board[sourceRank][sourceCol] = "*"

        # move was successful
        self.myTurn = False

    def getFENPrefixFromBoard(self):
        FENString = ""
        r = 0
        while r < 8:
            colString = ""
            c = 0
            while c < 8:
                if self.board[r][c] != "*":
                    colString += self.board[r][c]
                else:
                    blankCounter = 0
                    while c < 8 and self.board[r][c] == "*":
                        blankCounter += 1
                        c += 1
                    colString += str(blankCounter)
                    continue

                c += 1
            if r != 7:
                FENString += colString + "/"
            else:
                FENString += colString + " "

            r += 1
        return FENString

    def printBoard(self):
        for i in range(7, -1, -1):
            print(self.board[i])
        print()

    def getAIMove(self):
        return self.stockfish.get_best_move()


c = ChessEngine()
c.updateBoard()
c.printBoard()
# print(c.getAIMove())
print()
c.moveSourceToDest("e2", "e4")
c.moveSourceToDest("f2", "f4")
c.printBoard()
print(c.getFENPrefixFromBoard())
