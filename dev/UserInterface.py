import chess
import chess.svg
from tkinter import *
from PIL import Image, ImageTk
import os
import tksvg
from io import BytesIO

import cairosvg



inGame = False



GUI = Tk()
GUI.geometry("1000x500")
GUI.title("Robotic Chess Player")

programimage = Image.open("robo_arm.jpg")
iconimage = ImageTk.PhotoImage(programimage)
GUI.iconphoto(True,iconimage)
GUI.config()



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
def startGame():
    print("Started Game")
    inGame = True
    play.place_forget()
    end.place(x=300,y=0)


play.config(command=startGame)
play.place(x=0,y=0)


def endGame():
    print("Ended Game")
    inGame = False
    end.place_forget()
    play.place(x=0,y=0)

    
    

print("Printing Chess Board")
board = chess.Board("8/8/8/8/4N3/8/8/8 w - - 0 1")
chessBoardSvg = chess.svg.board(board,size=500)
pngdata = BytesIO()
cairosvg.svg2png(bytestring=chessBoardSvg.encode('utf-8'), write_to=pngdata) 
pngdata.seek(0)
image = Image.open(pngdata)
displayedboard = ImageTk.PhotoImage(image)
boardDraw = Canvas(GUI,width=500,height=500,bg="white")
boardDraw.pack()
boardDraw.create_image(250,250,image=displayedboard)

end.config(command=endGame)
GUI.mainloop() #opens window
