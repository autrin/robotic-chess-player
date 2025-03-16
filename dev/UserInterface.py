import chess
import chess.svg
from tkinter import *
from PIL import Image, ImageTk
import os
import tksvg
from io import BytesIO
import copy
import cairosvg
import time

global GUI 
inGame = False
#wrapper that contains all board information, including fen
class boardInfo:
    def __init__(self,displayed_board,fen_string,chess_board):
        self.display=Canvas(GUI,width=500,height=500,bg="white")
        self.boardImage=displayed_board
        self.fen = fen_string
        self.chessBoard = chess_board
    def UpdateFen(self,fen_new):
        self.fen=fen_new
    def GetFen(self):
        return self.fen
    def SetBoardImage(self,image_new):
        self.boardImage=image_new
    def GetBoardImage(self):
        return self.boardImage
    def DisplayBoard(self):
        self.display.pack()
        self.display.create_image(250,250,image=self.boardImage)
    def GetBoard(self):
        return self.chessBoard
        

GUI = Tk()
GUI.geometry("1000x500")
GUI.title("Robotic Chess Player")

programimage = Image.open("robo_arm.jpg")
iconimage = ImageTk.PhotoImage(programimage)
GUI.iconphoto(True,iconimage)
GUI.config()
#takes FEN string and returns board with that string
def getBoard(FEN):
    retBoard = chess.Board(FEN)
    return retBoard
#takes board and returns the image to be displayed
def getDisplayBoard(boardVal,arrowsVal = "hi"):
    chessBoardSvg = 0
    if arrowsVal == "hi":
        chessBoardSvg = chess.svg.board(boardVal,size=500)
    else:
        chessBoardSvg = chess.svg.board(boardVal,size=500,arrows = arrowsVal)
    pngdata = BytesIO()
    cairosvg.svg2png(bytestring=chessBoardSvg.encode('utf-8'), write_to=pngdata) 
    pngdata.seek(0)
    image = Image.open(pngdata)
    return ImageTk.PhotoImage(image)
    
#set default board
boarddata = getBoard("rnbqkbnr/pp1ppppp/8/2p5/4P3/5N2/PPPP1PPP/RNBQKB1R b KQkq - 1 2")
displayedboard = getDisplayBoard(boarddata)
Board = boardInfo(displayedboard,"rnbqkbnr/pp1ppppp/8/2p5/4P3/5N2/PPPP1PPP/RNBQKB1R b KQkq - 1 2",boarddata)
Board.DisplayBoard()

#BUTTON TO PLAY/START GAME
play = Button(GUI,
                text ="Play",
                font=('Times_New_Roman'),
                relief=RAISED,
                bd=5,
                padx=10,
                pady=10
                )
#BUTTON TO GO BACK TO HOME
end = Button(GUI, text ="Concede",
                font=('Times_New_Roman'),
                relief=RAISED,
                bd=5,
                padx=10,
                pady=10)
#BUTTON TO END TURN
endTurn = Button(GUI,
                text ="End Turn",
                font=('Times_New_Roman'),
                relief=RAISED,
                bd=5,
                padx=10,
                pady=10
                )
#sets inGame to true and shows stuff
def startGame():
    print("Started Game")
    inGame = True
    play.place_forget()
    endTurn.place(x=0,y=200)
    end.place(x=0,y=0)

play.config(command=startGame)
play.place(x=0,y=0)

#sets InGame to false and hides stuff
def endGame():
    print("Ended Game")
    inGame = False
    end.place_forget()
    endTurn.place_forget()
    play.place(x=0,y=0)
    chess.Board.reset(Board.board)
end.config(command=endGame)

#majority of code for getting data from arm will be in this method.
def finishTurn():

    CurBoardFEN = Board.fen
    startsquare = 0
    endsquare = 0
    newMove = 0
    #set new board
    boardTemp = chess.Board(CurBoardFEN)
    for move in boardTemp.legal_moves:
        startsquare = move.from_square
        endsquare = move.to_square
        newMove = move
        break
    #update screen with board displaying arrow for next move.
    arrowImage = getDisplayBoard(boardTemp,arrowsVal=[chess.svg.Arrow(startsquare,endsquare)])
    Board.SetBoardImage(arrowImage)
    Board.DisplayBoard()

    
    #update the screen instantly and wait a bit to display the move.
    GUI.update()
    GUI.after(1000)


    #set temp Board's FEN to the current board FEN after the new move
    boardTemp.push(newMove)
    Board.fen=boardTemp.board_fen()
    #update screen with board displaying last move
    newBoardImage = getDisplayBoard(boardTemp)
    Board.SetBoardImage(newBoardImage)
    Board.DisplayBoard()
endTurn.config(command= finishTurn)




def turnBoardIntoImage(board):
    chessBoardSvg = chess.svg.board(board,size=500)
    pngdata = BytesIO()
    cairosvg.svg2png(bytestring=chessBoardSvg.encode('utf-8'), write_to=pngdata) 
    pngdata.seek(0)
    image = Image.open(pngdata)
    displayedboard = ImageTk.PhotoImage(image)
    return displayedboard

#given two board states with a difference of one move, returns that move.
def getMoveFromTwoBoardStates(state1,state2):
    state1copy = copy.copy(state1)
    movefound = 0
    for move in state1copy.legal_moves:
        state1copy.push(move)
        if state1copy.board_fen()== state2.board_fen():
            movefound=move
        state1copy.pop(move)
    return movefound



GUI.mainloop() #opens window
