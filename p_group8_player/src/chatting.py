#!/usr/bin/python3

# IMPORTS
import rospy
from tkinter import *  # Import tkinter library

def main():
    rospy.init_node('chatting_node')
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            window = Tk()  # declare the window
            window.title('PSR TeamHunt p_group8 chat')  # set window title
            window.configure(width=500, height=300)  # set window width and height
            window.configure(bg='white')  # set window background color

            # if red is hunted from blue -> ex. text = 'R1: Oh no, run!' and text = 'B1: I am coming bro!'
            lbl = Label(window, text='R1: Oh no, run!', bg='White', fg='Red')  # not use label but something different
            # if blue catches red -> ex. text = 'B1: Game over bro!' and text = 'R1: :('

            lbl.place(x=20, y=10)

            winWidth = window.winfo_reqwidth()
            winwHeight = window.winfo_reqheight()
            posRight = int(window.winfo_screenwidth() / 2 - winWidth / 2)
            posDown = int(window.winfo_screenheight() / 2 - winwHeight / 2)
            window.geometry("+{}+{}".format(posRight, posDown))

            window.mainloop()

            rate.sleep()
        except rospy.ServiceException as e:
            print('Service call failed %s', e)

    # TODO: change line and create bar to scroll the text when chat is full
    # TODO: after defined somewhere in the program the boolean variables to se if the robot is chatching o is chatched implement the informations strings
    # TODO: if I close the chat, the chat as to remain closed


if __name__ == '__main__':
    main()
