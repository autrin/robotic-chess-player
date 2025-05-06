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

PlayerTime = 600
RobotTime = 600
global GUI
inGame = False
global isPlayerTurn
isPlayerTurn = False


# wrapper that contains all board information, including fen
class boardInfo:
    def __init__(self, displayed_board, fen_string, chess_board):
        self.display = Canvas(GUI, width=500, height=500, bg="white")
        self.boardImage = displayed_board  # image of board
        self.fen = fen_string  # fen string
        self.chessBoard = chess_board  # actual board object

    def UpdateFen(self, fen_new):
        self.fen = fen_new

    def GetFen(self):
        return self.fen

    def SetBoardImage(self, image_new):
        self.boardImage = image_new

    def GetBoardImage(self):
        return self.boardImage

    def DisplayBoard(self):
        self.display.pack()
        self.display.create_image(250, 250, image=self.boardImage)

    def HideBoard(self):
        self.display.pack_forget()
        self.display.delete("all")
        self.display.update()

    def GetBoard(self):
        return self.chessBoard

    def ResetBoard(self):
        self.fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"


# INITIALIZE GUI
GUI = Tk()
GUI.geometry("1000x500")
GUI.title("Robotic Chess Player")

programimage = Image.open("robo_arm.jpg")
iconimage = ImageTk.PhotoImage(programimage)
GUI.iconphoto(True, iconimage)
GUI.config()


# takes FEN string and returns board with that string
def getBoard(FEN):
    retBoard = chess.Board(FEN)
    return retBoard


# takes board and returns the image to be displayed
def getDisplayBoard(boardVal, arrowsVal="hi"):
    chessBoardSvg = 0
    if arrowsVal == "hi":
        chessBoardSvg = chess.svg.board(boardVal, size=500)
    else:
        chessBoardSvg = chess.svg.board(boardVal, size=500, arrows=arrowsVal)
    pngdata = BytesIO()
    cairosvg.svg2png(bytestring=chessBoardSvg.encode('utf-8'), write_to=pngdata)
    pngdata.seek(0)
    image = Image.open(pngdata)
    return ImageTk.PhotoImage(image)


# Robot Text
RobotText = "Robot's Turn"
RobotTurnText = Label(GUI, text=RobotText, font=("Arial", 20))

PlayerText = "Player's Turn"
PlayerTurnText = Label(GUI, text=PlayerText, font=("Arial", 20))

# set default board
global defaultfen
defaultfen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
boarddata = getBoard(defaultfen)
displayedboard = getDisplayBoard(boarddata)
Board = boardInfo(displayedboard, defaultfen, boarddata)

# BUTTON TO PLAY/START GAME
play = Button(GUI,
              text="Play",
              font=('Times_New_Roman'),
              relief=RAISED,
              bd=5,
              padx=10,
              pady=10
              )
# BUTTON TO GO BACK TO HOME
end = Button(GUI, text="Concede",
             font=('Times_New_Roman'),
             relief=RAISED,
             bd=5,
             padx=10,
             pady=10)
# BUTTON TO END TURN
endTurn = Button(GUI,
                 text="End Turn",
                 font=('Times_New_Roman'),
                 relief=RAISED,
                 bd=5,
                 padx=10,
                 pady=10
                 )
# BUTTON TO CHOOSE DIFFICULTY 1 FOR STOCKFISH
difficulty1 = Button(GUI,
                     text="800",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )

# BUTTON TO CHOOSE DIFFICULTY 2 FOR STOCKFISH
difficulty2 = Button(GUI,
                     text="1100",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )
# BUTTON TO CHOOSE DIFFICULTY 3 FOR STOCKFISH
difficulty3 = Button(GUI,
                     text="1400",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )
# BUTTON TO CHOOSE DIFFICULTY 4 FOR STOCKFISH
difficulty4 = Button(GUI,
                     text="1700",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )
# BUTTON TO CHOOSE DIFFICULTY 5 FOR STOCKFISH
difficulty5 = Button(GUI,
                     text="2000",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )
# BUTTON TO CHOOSE DIFFICULTY 6 FOR STOCKFISH
difficulty6 = Button(GUI,
                     text="2300",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )
# BUTTON TO CHOOSE DIFFICULTY 7 FOR STOCKFISH
difficulty7 = Button(GUI,
                     text="2700",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )
# BUTTON TO CHOOSE DIFFICULTY 8 FOR STOCKFISH
difficulty8 = Button(GUI,
                     text="3000",
                     font=('Times_New_Roman'),
                     relief=RAISED,
                     bd=5,
                     padx=10,
                     pady=10
                     )


def showHomepage():
    distance = 60
    play.place(x=0, y=0)
    difficulty1.place(x=200, y=distance * 0)
    difficulty2.place(x=200, y=distance * 1)
    difficulty3.place(x=200, y=distance * 2)
    difficulty4.place(x=200, y=distance * 3)
    difficulty5.place(x=200, y=distance * 4)
    difficulty6.place(x=200, y=distance * 5)
    difficulty7.place(x=200, y=distance * 6)
    difficulty8.place(x=200, y=distance * 7)


def hideHomepage():
    play.place_forget()
    difficulty1.place_forget()
    difficulty2.place_forget()
    difficulty3.place_forget()
    difficulty4.place_forget()
    difficulty5.place_forget()
    difficulty6.place_forget()
    difficulty7.place_forget()
    difficulty8.place_forget()


showHomepage()


# sets inGame to true and shows stuff
def startGame():
    print("Started Game")
    hideHomepage()
    global inGame
    inGame = True
    global isPlayerTurn
    isPlayerTurn = True

    RobotTurnText.place(x=875, y=100, anchor="center")
    PlayerTurnText.place(x=875, y=400, anchor="center")
    RobotTurnText.config(fg="black", font=("Arial", 20))
    PlayerTurnText.config(fg="red", font=("Arial", 20, "bold"))

    RobotTimeLabel.place(x=875, y=150, anchor="center")
    playerTimeLabel.place(x=875, y=350, anchor="center")
    RobotTimeLabel.config(fg="black", font=("Arial", 20))
    playerTimeLabel.config(fg="red", font=("Arial", 20, "bold"))

    global Board
    Board.DisplayBoard()
    gameTimer()
    endTurn.place(x=0, y=200)
    end.place(x=0, y=0)


play.config(command=startGame)


# sets InGame to false and hides stuff
def endGame():
    print("Ended Game")
    showHomepage()
    global inGame
    inGame = False
    global timerAfter
    GUI.after_cancel(timerAfter)
    end.place_forget()
    endTurn.place_forget()
    RobotTurnText.place_forget()
    PlayerTurnText.place_forget()
    playerTimeLabel.place_forget()
    RobotTimeLabel.place_forget()
    global Board
    Board.HideBoard()
    global PlayerTime
    PlayerTime = 600
    global RobotTime
    RobotTime = 600
    play.place(x=0, y=0)
    # resets board
    # chess.Board.reset(Board.GetBoard())
    boarddata = getBoard(defaultfen)
    displayedboard = getDisplayBoard(boarddata)
    Board.SetBoardImage(displayedboard)
    Board.UpdateFen(defaultfen)


end.config(command=endGame)

playerTimeLabel = Label(GUI, text=PlayerTime)
RobotTimeLabel = Label(GUI, text=RobotTime)


def gameTimer():
    global PlayerTime
    global RobotTime
    global isPlayerTurn
    global timerAfter
    if inGame == True:
        if PlayerTime > 0 and RobotTime > 0:
            timerAfter = GUI.after(1000, gameTimer)
            if isPlayerTurn == True:
                PlayerTime = PlayerTime - 1
                print(PlayerTime)
                playerTimeLabel.configure(text=PlayerTime)
                GUI.update()
            else:
                RobotTime = RobotTime - 1
                print(RobotTime)
                RobotTimeLabel.configure(text=RobotTime)
                GUI.update()
        else:
            endGame()
    else:
        GUI.after_cancel(timerAfter)


def startRobotTurn():
    global isPlayerTurn
    isPlayerTurn = False
    RobotTurnText.config(fg="red", font=("Arial", 20, "bold"))
    PlayerTurnText.config(fg="black", font=("Arial", 20))
    RobotTimeLabel.config(fg="red", font=("Arial", 20, "bold"))
    playerTimeLabel.config(fg="black", font=("Arial", 20))

    GUI.after(3000, endRobotTurn)


def endRobotTurn():
    global isPlayerTurn
    isPlayerTurn = True
    RobotTurnText.config(fg="black", font=("Arial", 20))
    PlayerTurnText.config(fg="red", font=("Arial", 20, "bold"))
    RobotTimeLabel.config(fg="black", font=("Arial", 20))
    playerTimeLabel.config(fg="red", font=("Arial", 20, "bold"))
    global TurnGoingOn
    TurnGoingOn = False


TurnGoingOn = False


# majority of code for getting data from arm will be in this method.
def finishTurn():
    global TurnGoingOn
    print("Checking:", TurnGoingOn)
    if TurnGoingOn == False:
        TurnGoingOn = True

        GUI.after(100, startRobotTurn)
        CurBoardFEN = Board.fen
        startsquare = 0
        endsquare = 0
        newMove = 0
        # set new board
        boardTemp = chess.Board(CurBoardFEN)
        for move in boardTemp.legal_moves:
            startsquare = move.from_square
            endsquare = move.to_square
            newMove = move
            break
        # update screen with board displaying arrow for next move.
        arrowImage = getDisplayBoard(boardTemp, arrowsVal=[chess.svg.Arrow(startsquare, endsquare)])
        Board.SetBoardImage(arrowImage)
        Board.DisplayBoard()

        # update the screen instantly and wait a bit to display the move.
        GUI.update()
        # set temp Board's FEN to the current board FEN after the new move
        boardTemp.push(newMove)
        Board.fen = boardTemp.board_fen()
        # update screen with board displaying last move
        newBoardImage = getDisplayBoard(boardTemp)
        Board.SetBoardImage(newBoardImage)
        Board.DisplayBoard()

        print("End of Turn:", TurnGoingOn)


endTurn.config(command=finishTurn)


def turnBoardIntoImage(board):
    chessBoardSvg = chess.svg.board(board, size=500)
    pngdata = BytesIO()
    cairosvg.svg2png(bytestring=chessBoardSvg.encode('utf-8'), write_to=pngdata)
    pngdata.seek(0)
    image = Image.open(pngdata)
    displayedboard = ImageTk.PhotoImage(image)
    return displayedboard


# given two board states with a difference of one move, returns that move.
def getMoveFromTwoBoardStates(state1, state2):
    state1copy = copy.copy(state1)
    movefound = 0
    for move in state1copy.legal_moves:
        state1copy.push(move)
        if state1copy.board_fen() == state2.board_fen():
            movefound = move
        state1copy.pop(move)
    return movefound


GUI.mainloop()  # opens window
