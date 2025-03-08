import chess
import chess.svg
from tkinter import *
from PIL import Image, ImageTk


    
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
def startGame():
    print("Started Game")
    inGame = True
    play.place_forget()
    end.place(x=300,y=0)


    
play.config(command=startGame)
play.place(x=0,y=0)

#BUTTON TO GO BACK TO HOME

end = Button(GUI, text ="Concede",
                font=('Times_New_Roman'),
                relief=RAISED,
                bd=5,
                padx=10,
                pady=10)
def endGame():
    print("Ended Game")
    inGame = False
    end.place_forget()
    play.place(x=0,y=0)

end.config(command=endGame)

GUI.mainloop() #opens window
