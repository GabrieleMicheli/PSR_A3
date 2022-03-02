#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from colorama import Fore, Style


class Chatting:
    def __init__(self):
        self.state_msg = None
        # Get parameters from YAML file
        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')

        # concatenate the player name inside a single string
        players_list = [*red_players_list, *green_players_list, *blue_players_list]

        for player in players_list:
            rospy.Subscriber('/' + player + '/state_msg', String, self.robot_state_callback)

    def robot_state_callback(self, state_msg):
        if 'R' in state_msg.data:
            print(Fore.RED + state_msg.data + Style.RESET_ALL)
        elif 'G' in state_msg.data:
            print(Fore.GREEN + state_msg.data + Style.RESET_ALL)
        else:
            print(Fore.BLUE + state_msg.data + Style.RESET_ALL)
        self.state_msg = state_msg.data

    def chat_creation(self):
        if not self.state_msg.__eq__(None):
            print(self.state_msg)


def main():
    rospy.init_node('chatting_node')  # init chatting node
    chatting = Chatting()

    while not rospy.is_shutdown():
        chatting.chat_creation()
    rospy.spin()


if __name__ == '__main__':
    main()

