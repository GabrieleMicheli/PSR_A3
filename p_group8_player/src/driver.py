#!/usr/bin/python3

import copy
import math
import cv2
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from driver_functions import *


class Driver():
    def __init__(self):
        # Defining parameters
        self.team = 'Not defined'
        self.prey = 'Not defined'
        self.hunter = 'Not defined'

        # Getting parameters
        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')
        self.debug = rospy.get_param('~debug')

        # Defining name of the robot, team and respective info
        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # removing slash
        self.name = self.name.strip('/driver')  # removing slash
        rospy.loginfo('My player name is ' + self.name)
        self.getTeam(red_players_list, green_players_list, blue_players_list)
        self.information(red_players_list, green_players_list, blue_players_list)

        # define a goal Pose to which the robot should move
        self.goal = PoseStamped()
        self.goal_active = False

        # Defining cv bridge
        self.cv_bridge = CvBridge()

        # Defining transformation buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Defining publishers and subscriber
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)
        self.camera_subscriber = rospy.Subscriber('/' + self.name + '/camera/rgb/image_raw', Image, self.cameraCallback)

    # function to check the players team - harcoded
    def getTeam(self, red_players_list, green_players_list, blue_players_list):

        if self.name in red_players_list:
            self.team = 'Red'
            self.prey = 'Blue'
            self.hunter = 'Green'
            self.team_color = [0, 0, 255]
            self.prey_color = [255, 0, 0]
            self.hunter_color = [0, 255, 0]
        elif self.name in green_players_list:
            self.team = 'Green'
            self.prey = 'Red'
            self.hunter = 'Blue'
            self.team_color = [0, 255, 0]
            self.prey_color = [0, 0, 255]
            self.hunter_color = [255, 0, 0]
        elif self.name in blue_players_list:
            self.team = 'Blue'
            self.prey = 'Green'
            self.hunter = 'Red'
            self.team_color = [255, 0, 0]
            self.prey_color = [0, 255, 0]
            self.hunter_color = [0, 0, 255]
        else:
            self.team = 'Joker'

    def information(self, red_players_list, green_players_list, blue_players_list):
        if self.team == 'Red':
            rospy.loginfo("My name is " + str(self.name) + ". I am team " + str(self.team) + ". I am hunting " + str(
                green_players_list) + " and fleeing from " + str(blue_players_list))
        elif self.team == 'Green':
            rospy.loginfo("My name is " + str(self.name) + ". I am team " + str(self.team) + ". I am hunting " + str(
                blue_players_list) + " and fleeing from " + str(red_players_list))
        elif self.team == 'Blue':
            rospy.loginfo("My name is " + str(self.name) + ". I am team " + str(self.team) + ". I am hunting " + str(
                red_players_list) + " and fleeing from " + str(green_players_list))
        else:
            rospy.loginfo('You are a joker and you can just annoy the others!')

    def goalReceivedCallback(self, goal_msg):
        rospy.loginfo('Received new goal')
        self.goal = goal_msg  # storing goal inside the class
        self.goal_active = True

    def sendCommandCallback(self):
        # Decision outputs a speed (linear Velocity) and an angle (angular Velocity)
        # input: goal
        # output: angle and speed

        # Verify if the goal was reached
        if self.goal_active:
            distance_to_goal = self.computeDistanceToGoal(self.goal)
            if distance_to_goal < 0.1:
                rospy.logwarn('I have achieved my goal')
                self.goal_active = False

        # Define driving behaviour according to the goal
        if self.goal_active:
            angle, speed = self.driveStraight(self.goal)
            if angle is None or speed is None:  # Something went wrong with the decision, let's stop
                angle = 0
                speed = 0
        else:
            angle = 1
            speed = 1

        # Build the command message (Twist) and publish it
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angle

        # self.publisher_command.publish(twist)

    def computeDistanceToGoal(self, goal):
        goal_present_time = copy.deepcopy(goal)
        goal_present_time.header.stamp = rospy.Time.now()

        target_frame = self.name + '/base_footprint'
        try:
            goal_in_base_link = self.tf_buffer.transform(goal_present_time, target_frame, rospy.Duration(1))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not transform goal from' + goal.header.frane.id + ' to ' + target_frame + '.')
            return None, None

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        distance = math.sqrt(x ** 2 + y ** 2)

        return distance

    def driveStraight(self, goal, min_speed=0.1, max_speed=0.5):

        goal_present_time = copy.deepcopy(goal)
        goal_present_time.header.stamp = rospy.Time.now()

        target_frame = self.name + '/base_footprint'
        try:
            goal_in_base_link = self.tf_buffer.transform(goal_present_time, target_frame, rospy.Duration(1))
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not transform goal from' + goal.header.frane.id + ' to ' + target_frame + '.')
            return None, None

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        angle = math.atan2(y, x)  # compute the angle

        # Define the linear speed based on the distance to the goal
        distance = math.sqrt(x ** 2 + y ** 2)
        speed = 0.5 * distance

        # saturate speed to minimum and maximum values
        speed = min(speed, max_speed)
        speed = max(speed, min_speed)

        return angle, speed

    def cameraCallback(self, msg):
        # Convert from message to cv2 image
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Get an ima

        # x, y, w, h = cv2.boundingRect(cnt)
        # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if self.debug:
            cv2.namedWindow(self.name)
            cv2.imshow(self.name, img)
            cv2.waitKey(1)




def main():
    # ------------------------------------------------------
    # Initialization
    # ------------------------------------------------------
    rospy.init_node('R1', anonymous=False)

    driver = Driver()
    rate = rospy.Rate(10)


    # ------------------------------------------------------
    # Execution
    # ------------------------------------------------------
    while not rospy.is_shutdown():
        driver.sendCommandCallback()
        rate.sleep()


if __name__ == '__main__':
    main()
