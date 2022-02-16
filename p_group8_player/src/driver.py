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
import numpy as np
import logging
from math import *
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry



class Driver():
    def __init__(self):
        # Defining parameters
        self.team = 'Not defined'
        self.prey = 'Not defined'
        self.hunter = 'Not defined'
        self.centroids = []

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
        self.laser_subscriber = rospy.Subscriber('/' + self.name + '/scan', LaserScan, self.lidarScanCallback)
        self.odom_sub = rospy.Subscriber(self.name + "/odom", Odometry, self.odomCallback)

        # Defining threshold limits for image processing masks
        self.blue_limits = {'B': {'max': 255, 'min': 100}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 50, 'min': 0}}
        self.red_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 255, 'min': 100}}
        self.green_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 255, 'min': 100}, 'R': {'max': 50, 'min': 0}}

        self.index_color = {'blue': 0, 'green': 1, 'red': 2}

        self.connectivity = 4

        # Laser Scan Parameters
        self.MAX_LIDAR_DISTANCE = 1.0
        self.COLLISION_DISTANCE = 0.14  # LaserScan.range_min = 0.1199999
        self.NEARBY_DISTANCE = 0.45

        self.ZONE_0_LENGTH = 0.4
        self.ZONE_1_LENGTH = 0.7

        self.ANGLE_MAX = 360 - 1
        self.ANGLE_MIN = 1 - 1
        self.HORIZON_WIDTH = 75

    # Function to check the players team - harcoded
    # Not hardcoded anymore what do you think?
    def getTeam(self, red_players_list, green_players_list, blue_players_list):

        if self.name in red_players_list:
            self.team = 'Red'
            self.prey = 'Blue'
            self.hunter = 'Green'
            self.team_players = red_players_list
            self.prey_team_players = green_players_list
            self.hunter_team_players = blue_players_list
        elif self.name in green_players_list:
            self.team = 'Green'
            self.prey = 'Red'
            self.hunter = 'Blue'
            self.team_players = green_players_list
            self.prey_team_players = blue_players_list
            self.hunter_team_players = red_players_list
        elif self.name in blue_players_list:
            self.team = 'Blue'
            self.prey = 'Green'
            self.hunter = 'Red'
            self.team_players = blue_players_list
            self.prey_team_players = red_players_list
            self.hunter_team_players = green_players_list
        else:
            self.team = 'Joker'

    # ------------------------------------------------------
    #             Terminal Printing Information
    # ------------------------------------------------------

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

    # ------------------------------------------------------
    # ------------------------------------------------------
    #               CallBack Functions
    # ------------------------------------------------------
    # ------------------------------------------------------

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

        target_frame = self.name + '/base_footrospy.loginfo'
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

        target_frame = self.name + '/base_footrospy.loginfo'
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

        # Convert subscribed image msg to cv2 image
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.debug:
            centroid = self.getCentroid(self.cv_image, 'Red')
            rospy.loginfo(str(centroid))
        # if self.debug:
        #     cv2.namedWindow(self.name)
        #     cv2.waitKey(1)
        #     # print(str(self.centroids[1][0]))
        #     cv2.putText(self.mask, str('hellooo'), org=(100, 100),
        #                 fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=5, color=(255, 0, 0), thickness=3)
        #     cv2.imshow(self.name, self.mask)
        #     rospy.loginfo('debug print')

    def getCentroid(self, image, team):
        """
            Create a mask with the largest blob of mask_original and return its centroid coordinates
            :param team: Team name - String
            :param image: Cv2 image - Uint8
            :return centroid: list of tuples with (x,y) coordinates of the largest object
            """

        if team == 'Red':
            self.mask = cv2.inRange(image, (
                self.red_limits['B']['min'], self.red_limits['G']['min'], self.red_limits['R']['min']),
                                    (self.red_limits['B']['max'], self.red_limits['G']['max'],
                                     self.red_limits['R']['max']))

        elif team == 'Green':
            self.mask = cv2.inRange(image, (
                self.green_limits['B']['min'], self.green_limits['G']['min'], self.green_limits['R']['min']),
                                    (self.green_limits['B']['max'], self.green_limits['G']['max'],
                                     self.green_limits['R']['max']))

        elif team == 'Blue':
            self.mask = cv2.inRange(image, (
                self.blue_limits['B']['min'], self.blue_limits['G']['min'], self.blue_limits['R']['min']),
                                    (self.blue_limits['B']['max'], self.blue_limits['G']['max'],
                                     self.blue_limits['R']['max']))

        # Extract results from mask
        results = cv2.connectedComponentsWithStats(self.mask, self.connectivity, ltype=cv2.CV_32S)
        no_labels = results[0]
        labels = results[1]
        stats = results[2]
        centroids = results[3]

        # Initialize variables
        maximum_area = 0
        largest_object_idx = 1
        player_detected = True
        object_area = 0

        for i in range(1, no_labels):
            object_area = stats[i, cv2.CC_STAT_AREA]
            if object_area > maximum_area:
                maximum_area = object_area
                largest_object_idx = i

        # Used only for eliminating noise detection
        if object_area < 30:
            player_detected = False

        # Append tuples of centroid coordinates if a player is detected
        if player_detected:
            x, y = centroids[largest_object_idx]
            x = int(x)
            y = int(y)
            centroid_coordinates = (x, y)
            # self.centroids.append(centroid_coordinates)



        # # Convert labels into uint8 and append it to list
        # biggest_object = (labels == largest_object_idx)
        # biggest_object = biggest_object.astype(np.uint8) * 255
        # biggest_objects.append(biggest_object)

        return centroid_coordinates

    def lidarScanCallback(self, msgScan):

        self.laser_subscriber = msgScan

        # Initializing distance and angles arrays
        distances = np.array([])
        angles = np.array([])

        for i in range(len(msgScan.ranges)):
            angle = degrees(i * msgScan.angle_increment)

            if msgScan.ranges[i] > self.MAX_LIDAR_DISTANCE:
                distance = self.MAX_LIDAR_DISTANCE

            elif msgScan.ranges[i] < msgScan.range_min:
                distance = msgScan.range_min
                # For real robot - protection
                if msgScan.ranges[i] < 0.01:
                    distance = self.MAX_LIDAR_DISTANCE

            else:
                distance = msgScan.ranges[i]

            distances = np.append(distances, distance)
            angles = np.append(angles, angle)

        # distances in [m], angles in [degrees]
        return (distances, angles)


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
