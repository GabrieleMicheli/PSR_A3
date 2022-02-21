#!/usr/bin/env python3

from tkinter import *  # Import tkinter library


def moveWindowCenter(window):
    winWidth = window.winfo_reqwidth()
    winwHeight = window.winfo_reqheight()
    posRight = int(window.winfo_screenwidth() / 2 - winWidth / 2)
    posDown = int(window.winfo_screenheight() / 2 - winwHeight / 2)
    window.geometry("+{}+{}".format(posRight, posDown))


def main():
    # declare the window
    window = Tk()
    # set window title
    window.title('TeamHunt chat')
    # set window width and height
    window.configure(width=500, height=300)
    # set window background color
    window.configure(bg='white')

    # if red is hunted from blue -> ex. text = 'R1: Oh no, run!' and text = 'B1: I am coming bro!'
    lbl = Label(window, text='R1: Oh no, run!', bg='White', fg='Red')  # not use label but something different
    # if blue catches red -> ex. text = 'B1: Game over bro!' and text = 'R1: :('

    lbl.place(x=20, y=10)

    moveWindowCenter(window)

    window.mainloop()

    # TODO: change line and create bar to scroll the text when chat is full
    # TODO: after defined somewhere in the program the boolean variables to se if the robot is chatching o is chatched implement the informations strings


if __name__ == '__main__':
    main()
