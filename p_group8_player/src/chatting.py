#!/usr/bin/python3

# IMPORTS
import rospy
from tkinter import *  # Import tkinter library
from tkinter import ttk


def create_window(title, width, height, backGroundColour):
    window = Tk()  # declare the window
    window.title(title)  # set window title
    window.configure(width=width, height=height, bg=backGroundColour)  # set window width, height and back ground colour

    return window


def create_frame(window, backGroundColour):
    frame = Frame(window)
    frame.pack(fill=BOTH, expand=1)
    frame.configure(bg=backGroundColour)  # set frame background color
    return frame


def create_canvas(frame, backGroundColour):
    canvas = Canvas(frame)
    canvas.pack(side=LEFT, fill=BOTH, expand=1)
    canvas.configure(bg=backGroundColour)
    return canvas


def addScrollbarToCanvas(frame, canvas):
    scrollbar = ttk.Scrollbar(frame, orient=VERTICAL, command=canvas.yview)
    scrollbar.pack(side=RIGHT, fill=Y)
    return scrollbar


def clear_label(label):
    label.destroy()


def main():
    rospy.init_node('chatting_node')  # init chatting node

    # defining window properties
    window_title = 'PSR TeamHunt p_group8 chat'
    width = 500
    height = 800
    backGroundColour = 'white'

    window = create_window(window_title, width, height, backGroundColour)  # creating a chat window

    main_frame = create_frame(window, backGroundColour)  # creating a main window

    canvas = create_canvas(main_frame, backGroundColour)  # creating a canvas

    scrollbar = addScrollbarToCanvas(main_frame, canvas)  # adding a scrollbar to canvas

    # configure the canvas
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

    # create another frame inside the canvas
    second_frame = Frame(canvas)
    second_frame.configure(bg='white')  # set window background color

    # add that new frame to a window in the canvas
    canvas.create_window((0, 0), window=second_frame, anchor="nw")

    label = Label(second_frame, text='R1: Oh no, run!\n', bg='White', fg='Red')  # creating a label
    label.grid(row=1, column=1)  # label positioning

    # create a button to clear the chat
    clear_button = Button(second_frame, text='CLEAR', command=lambda: clear_label(label))
    clear_button.grid(row=10000 - 2, column=10)  # button positioning

    def handleProtocol():  # handle WM_DELETE_WINDOW event
        window.destroy()  # deleting the chat windows

    window.protocol("WM_DELETE_WINDOW", handleProtocol)

    window.mainloop()


# TODO: By actions, print a specific message!

if __name__ == '__main__':
    main()
