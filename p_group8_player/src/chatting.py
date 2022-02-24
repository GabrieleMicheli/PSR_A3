#!/usr/bin/python3

# IMPORTS
import rospy
from tkinter import *  # Import tkinter library
from tkinter import ttk

def main():
    rospy.init_node('chatting_node') # init chatting node

    window = Tk()  # declare the window
    window.title('PSR TeamHunt p_group8 chat')  # set window title
    window.configure(width=500, height=300,bg='white')  # set window width and height

    # create a main frame
    main_frame = Frame(window)
    main_frame.pack(fill=BOTH, expand=1)
    main_frame.configure(bg='white')  # set window background color

    # create a canvas
    my_canvas = Canvas(main_frame)
    my_canvas.pack(side=LEFT, fill=BOTH, expand=1)
    my_canvas.configure(bg='white')

    # add a scrollbar to the canvas
    my_scrollbar = ttk.Scrollbar(main_frame, orient=VERTICAL, command=my_canvas.yview)
    my_scrollbar.pack(side=RIGHT, fill=Y)

    # configure the canvas
    my_canvas.configure(yscrollcommand=my_scrollbar.set)
    my_canvas.bind('<Configure>', lambda e: my_canvas.configure(scrollregion=my_canvas.bbox("all")))

    # create another frame inside the canvas
    second_frame = Frame(my_canvas)
    second_frame.configure(bg='white')  # set window background color

    # add that new frame to a window in the canvas
    my_canvas.create_window((0, 0), window=second_frame, anchor="nw")

    label = Label(second_frame, text='R1: Oh no, run!\n', bg='White', fg='Red').grid(row=1, column=1)

    def handleProtocol(): # handle WM_DELETE_WINDOW event
        window.destroy()

    window.protocol("WM_DELETE_WINDOW", handleProtocol)

    window.mainloop()

# TODO: change line
# TODO: after defined somewhere in the program the boolean variables to se if the robot is chatching o is chatched implement the informations strings
# TODO: create a button to clear the chat

if __name__ == '__main__':
    main()
