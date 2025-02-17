from stockfish import Stockfish
import re

class ChessEngine:

    def __init__(self,depth = 20,
                 level=15, side = 'w', 
                 enginePath = "/home/jshim/catkin_ws/src/my_package/src/stockfish/stockfish-ubuntu-x86-64-avx512"):
        self.depth = max(20,depth)
        self.level = max(20,level)
        self.side = side
        
        self.stockfish = Stockfish(enginePath)
        self.stockfish.set_depth(self.depth)
        self.stockfish.set_skill_level(self.level)
        self.fenPos = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1" # so that we can mutate
        self.stockfish.set_fen_position(self.fenPos) 
        
    
    def setFenPos(self):
        self.stockfish.set_fen_position(self.fenPos)    

    #needed for later on. For manipulation of the string
    def getRankStartEndPos(self,rank):
        lis = self.fenPos.split(" ")
        lis = lis[0].replace(" ", "")
        lis = lis.split("/")
        print(lis)
        startPos = 0

        for i in range(0, rank-1):
            startPos += (len(lis[i])+1)

        return startPos, startPos + len(lis[rank-1])-1


    
    def printBoard(self):
        boardString = ""
        for r in range (1,9):
            sp, ep = self.getRankStartEndPos(r)
            print(f"{sp,ep}")
            rowString = ""
            for c in range(sp,ep+1):
                if self.fenPos[c].isdigit():
                    for i in range(0, int(self.fenPos[c])):
                        rowString += "* "
                else:
                    rowString += (self.fenPos[c] + " ")
            boardString += (rowString +"\n") 
        print(boardString)

c = ChessEngine()
c.printBoard()