#!/usr/bin/python3

import rospy
from std_msgs.msg import String

class Chatting:
    def __init__(self):
        # Get parameters from YAML file
        self.state_msg = None
        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')

        # concatenate the player name inside a single string
        players_list = [*red_players_list, *green_players_list, *blue_players_list]

        for player in players_list:
            robot_state_subscriber = rospy.Subscriber('/' + player + '/state_msg', String, self.robot_state_callback)

    def robot_state_callback(self, state_msg):
        # if
        print(state_msg.data)
        self.state_msg = state_msg.data

    def chat_creation(self): # NOW is printing messages only when the state changes
        if not self.state_msg.__eq__(None):
            print(self.state_msg)


# TODO: By actions, print a specific message!


def main():
    rospy.init_node('chatting_node')  # init chatting node
    chatting = Chatting()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        chatting.chat_creation()
    rospy.spin()

if __name__ == '__main__':
    main()
