#!/usr/bin/python3

import rospy
from tkinter import *  # Import tkinter library
from tkinter import ttk
from std_msgs.msg import String

class Chatting():
    def __init__(self):
        # VARIABLE INITIALIZATION
        self.window = None  # declare the window
        self.main_frame = None
        self.second_frame = None
        self.canvas = None
        self.label = None
        self.state_msg = 'Welcome'
        self.player_label = None
        self.clear_button = None

        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')

        # concatenate the player name inside a single string
        players_list = [*red_players_list, *green_players_list, *blue_players_list]

        # SUBSCRIBER
        for player in players_list:
            self.robot_state_subscriber = rospy.Subscriber('/' + player + '/state', String, self.robot_state_callback)

    def windowConfiguration(self, title, width, height, bg_colour):
        self.window = Tk() # declare the window
        self.window.title(title)  # set window title
        self.window.configure(width=width, height=height,bg=bg_colour)  # set window width, height and back ground colour

    def mainFrameConfiguration(self, bg_colour):
        self.main_frame = Frame(self.window)
        self.main_frame.pack(fill=BOTH, expand=1)
        self.main_frame.configure(bg=bg_colour)  # set frame background color

    def canvasConfiguration(self, bg_colour):
        self.canvas = Canvas(self.main_frame)
        self.canvas.pack(side=LEFT, fill=BOTH, expand=1)
        self.canvas.configure(bg=bg_colour)

    def addScrollbarToCanvas(self):
        self.scrollbar = ttk.Scrollbar(self.main_frame, orient=VERTICAL, command=self.canvas.yview)
        self.scrollbar.pack(side=RIGHT, fill=Y)

    def secondFrameConfiguration(self, bg_colour):
        self.second_frame = Frame(self.canvas)
        self.second_frame.configure(bg='white')  # set window background color

    def clearLabel(self):
        self.label.destroy()

    def createChat(self, title, width, height, window_bg_colour, main_frame_bg_colour, canvas_bg_colour,
                         second_frame_bg_colour):
        self.windowConfiguration(title, width, height, window_bg_colour)
        self.mainFrameConfiguration(main_frame_bg_colour) # main_frame
        self.canvasConfiguration(canvas_bg_colour)
        self.addScrollbarToCanvas()

        # configure the canvas
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.bind('<Configure>', lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))

        self.secondFrameConfiguration(second_frame_bg_colour) # create another frame inside the canvas

        self.canvas.create_window((0, 0), window=self.second_frame, anchor="nw") # add the new frame to a window in the canvas
    #
    # def chattingCallback(self):
        # self.getLabel(self.robot_state_subscriber)

        self.label = Label(self.second_frame, text=self.state_msg, bg='White', fg='Red')  # creating a label
        self.label.grid(row=1, column=1)  # label positioning

        # create a button to clear the chat
        self.clear_button = Button(self.second_frame, text='CLEAR', command=lambda: self.clearLabel())
        self.clear_button.grid(row=10000 - 2, column=10)  # button positioning

        def handleProtocol():  # handle WM_DELETE_WINDOW event
            self.window.destroy()  # deleting the chat windows
            rospy.signal_shutdown('Closing windows')
            self.stop_flag = True

        self.window.protocol("WM_DELETE_WINDOW", handleProtocol)
        # self.window.after(0, self.chattingCallback())
        self.window.mainloop()

    def robot_state_callback(self, state_msg):
        if 'R1' in state_msg.data or 'R2' in state_msg.data or 'R3' in state_msg.data:
            print(state_msg.data)
        elif 'G1' in state_msg.data or 'G2' in state_msg.data or 'G3' in state_msg.data:
            print(state_msg.data)
        elif 'B1' in state_msg.data or 'B2' in state_msg.data or 'B3' in state_msg.data:
            print(state_msg.data)
        self.state_msg = state_msg.data

def main():
    rospy.init_node('chatting_node') # init chatting node
    # DEFINING WINDOW PROPERTIES
    window_title = 'PSR TeamHunt p_group8 chat'; width = 500; height = 800
    window_bg_colour = main_frame_bg_colour = second_frame_bg_colour = canvas_bg_colour = 'white'

    chatting = Chatting()

    chatting.createChat(window_title, width, height, window_bg_colour, main_frame_bg_colour, canvas_bg_colour,
                              second_frame_bg_colour)

    # rate = rospy.Rate(1)

    # while not rospy.is_shutdown():
    # chatting.chattingCallback()
    # rate.sleep()


if __name__ == '__main__':
    main()