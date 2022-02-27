#!/usr/bin/python3

# IMPORTS
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
        self.label_text = None
        self.player_label = None
        self.robot_state = None
        self.robot_state_list = []

        self.clear_button = None
        self.stop_flag = False
        self.robot_state_changed = False # boolean variable to check if the robot state is changed

        # SUBSCRIBER
        self.robot_state_subscriber = None

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

    def robotStateCallback(self, robot_state_msg):
        rospy.loginfo(robot_state_msg.data) # print /R1/state
        # return robot_state_msg.data

    # def robotStateCallback(self, robot_state_msg):
    #     # rospy.loginfo(rospy.get_caller_id() + 'I heard' + robot_state_msg.data)  # NOT WORKING: probably because inside a for loop?
    #     return robot_state_msg.data

    def getRobotStates(self):
        # Getting robot names
        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')
        players_list = [*red_players_list, *green_players_list, *blue_players_list]

        for i in range(players_list):
            self.robot_state = rospy.Subscriber('/' + players_list[i] + '/' + 'state', String, self.robotStateCallback)
        #     print
        #
        # print(self.robot_state_list)

    def robotStateChangeControl(self, actual_robot_state, old_robot_state):
        if actual_robot_state != old_robot_state:
            # rospy.loginfo('Robot status changed')
            self.robot_state_changed = True
        else:
            self.robot_state_changed = False

    def getLabel(self, robot_state_msg):
        if robot_state_msg.__eq__('wait'):
            self.label_text = 'What a deadly bore'
        elif robot_state_msg.__eq__('attack'):
            self.label_text = 'I am very hungry'
        elif robot_state_msg.__eq__('flee'):
            self.label_text = 'Oh no, I have to run'
        else:
            self.label_text = 'Walls everywhere in this game'

    def chattingCallback(self, title, width, height, window_bg_colour, main_frame_bg_colour, canvas_bg_colour,
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

        while not self.stop_flag:

            # self.robot_state_subscriber = rospy.Subscriber('/R1/state', String, self.robotStateCallback)

            # self.robotStateChangeControl(old_robot_state)

            # if self.robotStateChangeControl(old_robot_state):
            #     print(self.robot_state_subscriber)
            #     print('Status changed')
            # else:
            #     print('Status not changed')
            #
            # old_robot_state = self.robot_state_subscriber

            self.getLabel(self.robot_state_subscriber)
            self.label = Label(self.second_frame, text=self.label_text, bg='White', fg='Red')  # creating a label
            self.label.grid(row=1, column=1)  # label positioning

            # create a button to clear the chat
            self.clear_button = Button(self.second_frame, text='CLEAR', command=lambda: self.clearLabel())
            self.clear_button.grid(row=10000 - 2, column=10)  # button positioning

            def handleProtocol():  # handle WM_DELETE_WINDOW event
                self.window.destroy()  # deleting the chat windows
                self.stop_flag = True

            self.window.protocol("WM_DELETE_WINDOW", handleProtocol)

            self.window.mainloop()


        # TODO: By actions, print a specific message!

def main():

    rospy.init_node('chatting_node') # init chatting node

    # DEFINING WINDOW PROPERTIES
    window_title = 'PSR TeamHunt p_group8 chat'; width = 500; height = 800
    window_bg_colour = main_frame_bg_colour = second_frame_bg_colour = canvas_bg_colour = 'white'

    chatting = Chatting()
    chatting.chattingCallback(window_title, width, height, window_bg_colour, main_frame_bg_colour, canvas_bg_colour, second_frame_bg_colour)

if __name__ == '__main__':
    main()
