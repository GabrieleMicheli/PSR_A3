#!/usr/bin/python3

import rospy
from std_msgs.msg import String


class Chatting:
    def __init__(self):
        # VARIABLE INITIALIZATION
        self.old_robot_state = None
        self.actual_robot_state = None
        self.robot_state_msg = None # string to print inside the chat with a robot msg
        self.robot_state_changed = False  # boolean variable to check if the robot state is changed

        # Get parameters from YAML file
        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')

        # concatenate the player name inside a single string
        players_list = [*red_players_list, *green_players_list, *blue_players_list]

        # for player in players_list:
        self.robot_state_subscriber = rospy.Subscriber('/' + players_list[0] + '/state', String, self.robot_state_callback)

    def robot_state_callback(self, robot_state_msg):
    #     rospy.loginfo('Working') ###
        self.old_robot_state = self.actual_robot_state
        self.actual_robot_state = robot_state_msg.data
        if self.actual_robot_state != self.old_robot_state:
            # rospy.loginfo('Robot status changed')
            self.robot_state_changed = True
            return True
        else:
            # rospy.loginfo('Robot status not changed')
            self.robot_state_changed = False
            return False

    def robot_state_changing_control(self):
        if self.actual_robot_state != self.old_robot_state:
            # rospy.loginfo('Robot status changed')
            self.robot_state_changed = True
            return True
        else:
            # rospy.loginfo('Robot status not changed')
            self.robot_state_changed = False
            return False

    def get_robot_state_msg(self):
        if self.actual_robot_state.__eq__('wait'):
            # self.robot_state_msg = 'What a deadly bore'
            robot_state_msg = 'What a deadly bore'
        elif self.actual_robot_state.__eq__('attack'):
        # +    self.robot_state_msg = 'I am very hungry'
            robot_state_msg = 'I am very hungry'
        elif self.actual_robot_state.__eq__('flee'):
            # self.robot_state_msg = 'Oh no, I have to run'
            robot_state_msg = 'Oh no, I have to run'
        else:
            # self.robot_state_msg = 'Walls everywhere in this game'
            robot_state_msg = 'Walls everywhere in this game'
        return robot_state_msg
        # print(self.robot_state_msg)

    def chat_creation(self):
        # if self.robot_state_changed:
        if self.robot_state_changing_control():
            print(self.get_robot_state_msg())
            # print('hola')

# TODO: By actions, print a specific message!


def main():

    rospy.init_node('chatting_node')  # init chatting node
    chatting = Chatting()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # chatting.chatting+_callback()
        chatting.chat_creation()
        # rospy.sleep(1)
    # rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
