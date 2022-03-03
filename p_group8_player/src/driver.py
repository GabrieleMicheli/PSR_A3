#!/usr/bin/python3

import copy
import rospy
import std_msgs
from std_msgs.msg import String
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from driver_functions import *
# import OptimizationUtils.utilities as opt_utilities
# import atom_core.atom
import numpy as np
import math
from math import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Image, LaserScan, CameraInfo
from sensor_msgs import point_cloud2
from colorama import Fore, Back, Style
from termcolor import cprint
from prettytable import PrettyTable
from itertools import groupby
from operator import itemgetter
import random
import tf
import scipy.spatial.transform as transf


class Driver:

    def __init__(self):

        # <---------------------------------------------------------------------------------------------------------->
        # <--------------------------------Variable Initialization--------------------------------------------------->
        # <---------------------------------------------------------------------------------------------------------->
        self.old_state = 'None'
        self.state_msg = None
        self.actual_state = 'None'
        self.team = 'Not defined'
        self.prey = 'Not defined'
        self.hunter = 'Not defined'
        self.team_players = 'Not defined'
        self.hunter_team_players = 'Not defined'
        self.prey_team_players = 'Not defined'
        self.cv_image = []
        self.centroid_hunter = (0, 0)
        self.centroid_prey = (0, 0)
        self.centroid_teammate = (0, 0)
        self.state = 'wait'
        self.distance_hunter_to_prey = 0
        self.wall_avoiding_angle = 0
        self.linear_vel_to_wait = 0
        self.linear_vel_to_attack = 0
        self.angular_vel_to_wait = 0
        self.angular_vel_to_attack = 0
        self.linear_vel_to_flee = 0
        self.angular_vel_to_flee = 0
        self.linear_vel_to_avoid_wall = 0
        self.angular_vel_to_avoid_wall = 0
        self.linear_vel_to_avoid_teammate = 0
        self.angular_vel_to_avoid_teammate = 0
        self.min_range_detected = 0
        self.height = 0
        self.width = 0
        self.odom = None
        self.position = (0, 0)
        self.average_gap = 0
        self.min_range_angle = 0
        self.min_range_point = (0, 0, 0)
        self.exist_wall = False
        self.lidar_points = []
        self.camera_info_exist = False
        self.lidar_pixels = []
        self.pts_in_image = []
        self.lidar_pixels_in_image = []
        self.coord_hunter = (0, 0)
        self.coord_prey = (0, 0)
        self.coord_teammate = (0, 0)

        # Getting parameters
        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')
        self.debug = rospy.get_param('~debug')
        self.navigation = rospy.get_param('~navigation')

        # Defining name of the robot, team and respective info
        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # removing slash
        self.name = self.name.strip('/driver')  # removing slash

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

        # Defining threshold limits for image processing masks
        self.blue_limits = {'B': {'max': 255, 'min': 100}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 50, 'min': 0}}
        self.red_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 255, 'min': 100}}
        self.green_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 255, 'min': 100}, 'R': {'max': 50, 'min': 0}}

        # Other parameters
        self.connectivity = 4
        self.laser_thresh = 1.2
        self.color_marker1 = 1.0
        self.color_marker0 = 0
        # <---------------------------------------------------------------------------------------------------------->
        # <-----------------------------Publishers and Subscribers--------------------------------------------------->
        # <---------------------------------------------------------------------------------------------------------->
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback,
                                                queue_size=1)
        self.camera_subscriber = rospy.Subscriber('/' + self.name + '/camera/rgb/image_raw', Image, self.cameraCallback,
                                                  queue_size=1)
        self.laser_subscriber = rospy.Subscriber('/' + self.name + '/scan', LaserScan, self.lidarScanCallback,
                                                 queue_size=1)
        # self.clustering_subscriber = rospy.Subscriber('/' + self.name + '/scan', LaserScan, self.lidarClustering,
        #                                               queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odomPositionCallback,
                                                queue_size=1)

        self.subscriber_camera_info = rospy.Subscriber('/' + self.name + '/camera/rgb/camera_info', CameraInfo,
                                                       self.getCameraInfoCallback, queue_size=1)
        # self.referee_subscriber = rospy.Subscriber('/winner', String, self.callbackPodium, queue_size=1)
        # publishing the robot state: 'wait', 'attack', 'flee' and 'avoid_wall'
        self.robot_state_publisher = rospy.Publisher('/' + self.name + '/state_msg', String, queue_size=1)

        self.publisher_laser_distance = rospy.Publisher('/' + self.name + '/point_cloud', PointCloud2, queue_size=1)
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)
        self.clustering_lidar = rospy.Publisher('/' + self.name + '/marker_array', MarkerArray, queue_size=1)
        # self.publisher_point_cloud2 = rospy.Publisher('/' + self.name + '/point_cloud', PointCloud2)
        # self.laser_scan_subscriber = rospy.Subscriber('/' + self.name + '/scan', LaserScan, self.laser_scan_callback)

    # <---------------------------------------------------------------------------------------------------------->
    # <-----------------------------------Teams Setup------------------------------------------------------------>
    # <---------------------------------------------------------------------------------------------------------->
    def getTeam(self, red_players_list, green_players_list, blue_players_list):
        if self.name in red_players_list:
            self.team = 'Red'
            self.prey = 'Green'
            self.hunter = 'Blue'
            self.team_players = red_players_list
            self.prey_team_players = green_players_list
            self.hunter_team_players = blue_players_list
        elif self.name in green_players_list:
            self.team = 'Green'
            self.prey = 'Blue'
            self.hunter = 'Red'
            self.team_players = green_players_list
            self.prey_team_players = blue_players_list
            self.hunter_team_players = red_players_list
        elif self.name in blue_players_list:
            self.team = 'Blue'
            self.prey = 'Red'
            self.hunter = 'Green'
            self.team_players = blue_players_list
            self.prey_team_players = red_players_list
            self.hunter_team_players = green_players_list
        else:
            self.team = 'Joker'

    # <---------------------------------------------------------------------------------------------------------->
    # <--------------------------------Terminal Printing information--------------------------------------------->
    # <---------------------------------------------------------------------------------------------------------->

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

    # <---------------------------------------------------------------------------------------------------------->
    # <------------------------------------Callback functions---------------------------------------------------->
    # <---------------------------------------------------------------------------------------------------------->

    def goalReceivedCallback(self, goal_msg):
        rospy.loginfo('Received new goal')
        self.goal = goal_msg  # storing goal inside the class
        self.goal_active = True

    def odomPositionCallback(self, odomMsg):
        self.odom = odomMsg
        x = odomMsg.pose.pose.position.x
        y = odomMsg.pose.pose.position.y
        self.position = (x, y)

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

        if not self.navigation:
            self.decisionMaking()
            self.takeAction()
            twist = Twist()

            if self.state == 'wait':
                twist.linear.x = self.linear_vel_to_wait
                twist.angular.z = self.angular_vel_to_wait
            elif self.state == 'attack':
                twist.linear.x = self.linear_vel_to_attack
                twist.angular.z = self.angular_vel_to_attack
            elif self.state == 'flee':
                twist.linear.x = self.linear_vel_to_flee
                twist.angular.z = self.angular_vel_to_flee
            elif self.state == 'avoid_wall':
                twist.linear.x = self.linear_vel_to_avoid_wall
                twist.angular.z = self.angular_vel_to_avoid_wall
            # elif self.state == 'avoid_teammate':
            #     twist.linear.x = self.linear_vel_to_avoid_teammate
            #     twist.angular.z = self.angular_vel_to_avoid_teammate

            elif self.goal_active:
                twist.linear.x = speed
                twist.angular.z = angle

            self.publisher_command.publish(twist)

    def computeDistanceToGoal(self, goal):
        goal_present_time = copy.deepcopy(goal)
        goal_present_time.header.stamp = rospy.Time.now()

        target_frame = self.name + '/base_footprint'
        try:
            goal_in_base_link = self.tf_buffer.ltransform(goal_present_time, target_frame, rospy.Duration(1))
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

    # ------------------------------------------------------
    #               cameraCallback function
    # ------------------------------------------------------
    def cameraCallback(self, msg):

        # Convert subscribed image msg to cv2 image
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.height = self.cv_image.shape[0]
        self.width = self.cv_image.shape[1]

        # Calling getCentroid function and extracting hunter centroid coordinates and his mask
        centroid_hunter, frame_hunter = self.getCentroid(self.cv_image, self.hunter)
        (x_hunter, y_hunter) = centroid_hunter
        self.centroid_hunter = (x_hunter, y_hunter)

        # Calling getCentroid function and extracting prey centroid coordinates and his mask
        centroid_prey, frame_prey = self.getCentroid(self.cv_image, self.prey)
        (x_prey, y_prey) = centroid_prey
        self.centroid_prey = (x_prey, y_prey)

        # Calling getCentroid function and extracting teammate centroid coordinates and his mask
        centroid_teammate, frame_teammate = self.getCentroid(self.cv_image, self.team)
        (x_teammate, y_teammate) = centroid_teammate
        self.centroid_teammate = (x_teammate, y_teammate)

        self.decisionMaking()
        # self.printScores()
        # podium_img = callbackPodium('podium.jpg')
        if self.debug:
            if (x_hunter, y_hunter) != (0, 0):
                cv2.putText(frame_hunter, 'Hunter!', org=(x_hunter, y_hunter),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=(0, 0, 255), thickness=5)

            if (x_prey, y_prey) != (0, 0):
                cv2.putText(frame_prey, 'Prey!', org=(x_prey, y_prey),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=(0, 255, 0), thickness=5)

            # Merge the prey and hunter masks and plot it
            alpha = 0.5
            beta = (1.0 - alpha)
            hunters_n_preys = cv2.addWeighted(frame_hunter, alpha, frame_prey, beta, 0.0)
            # hunters_n_preys = cv2.bitwise_or(frame_prey, frame_hunter)
            cv2.putText(hunters_n_preys, str(self.state) + '' + str(self.exist_wall), (0, self.height - 50),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0), thickness=5)

            for p in self.lidar_pixels_in_image:
                x_lidar = round(int(p[0]), 2)
                y_lidar = round(int(p[1]), 2)
                cv2.circle(hunters_n_preys, (x_lidar, y_lidar), 2, (0, 0, 255), -1)

            # rospy.loginfo('hello')
            # # rospy.loginfo(self.points_in_camera[0])
            # rospy.loginfo(self.points_in_camera[0][:, 0][0])
            # rospy.loginfo(self.points_in_camera[0][:, 0][1])
            #
            #
            # cv2.circle(hunters_n_preys, (x_lidar, y_lidar), 5, (255,0 , 0), -1)

            cv2.namedWindow(self.name)
            cv2.imshow(self.name, hunters_n_preys)
            cv2.waitKey(1)

    # ------------------------------------------------------
    #               getCentroid function
    # ------------------------------------------------------
    def getCentroid(self, image, team):
        """
            Create a mask with the largest blob of mask_original and return its centroid coordinates
            :param team: Team name - String
            :param image: Cv2 image - Uint8
            :return centroid: list of tuples with (x,y) coordinates of the largest object
            :return player_identified: Cv2 image mask - Uint8
        """

        player_identified = np.copy(image)

        if team == 'Red':
            mask = cv2.inRange(image, (
                self.red_limits['B']['min'], self.red_limits['G']['min'], self.red_limits['R']['min']),
                               (self.red_limits['B']['max'], self.red_limits['G']['max'],
                                self.red_limits['R']['max']))

        elif team == 'Green':
            mask = cv2.inRange(image, (
                self.green_limits['B']['min'], self.green_limits['G']['min'], self.green_limits['R']['min']),
                               (self.green_limits['B']['max'], self.green_limits['G']['max'],
                                self.green_limits['R']['max']))

        elif team == 'Blue':
            mask = cv2.inRange(image, (
                self.blue_limits['B']['min'], self.blue_limits['G']['min'], self.blue_limits['R']['min']),
                               (self.blue_limits['B']['max'], self.blue_limits['G']['max'],
                                self.blue_limits['R']['max']))

        # Extract results from mask
        results = cv2.connectedComponentsWithStats(mask, self.connectivity, ltype=cv2.CV_32S)
        no_labels = results[0]
        labels = results[1]
        stats = results[2]
        centroids = results[3]

        # Initialize variables
        maximum_area = 0
        largest_object_idx = 1
        object_area = 0
        centroid = (0, 0)

        for i in range(1, no_labels):
            object_area = stats[i, cv2.CC_STAT_AREA]
            if object_area > maximum_area:
                maximum_area = object_area
                largest_object_idx = i

        # Used only for eliminating noise detection
        if object_area < 1:
            player_detected = False

        else:
            player_detected = True

        # Append tuples of centroid coordinates if a player is detected
        if player_detected:
            # Extract the centroid with the largest object idx
            x, y = centroids[largest_object_idx]
            x = int(x)
            y = int(y)
            centroid = (x, y)

            # Mask the largest object and paint with green
            largest_object = (labels == largest_object_idx)
            largest_object = largest_object.astype(np.uint8) * 255
            player_identified[largest_object == 255] = (255, 255, 255)

            # Identify in the image the centroid with red
            player_identified = cv2.circle(player_identified, (x, y), 15, (0, 0, 255), -1)

        return centroid, player_identified

    # ------------------------------------------------------
    #               decisionMaking function
    # ------------------------------------------------------
    def decisionMaking(self):
        """
           Creates the different states of the robot action
           :param centroid_prey: (x,y) - Tuple
           :param centroid_hunter: (x,y) - Tuple
           :param wall_avoiding_angle : angle in radians
           :param distance_hunter_to_prey : distance in meters
           :return state : state of robot - String
        """

        dist_to_hunter = sqrt((self.coord_hunter[0] ** 2 + self.coord_hunter[1]) ** 2)
        dist_to_prey = sqrt((self.coord_prey[0] ** 2 + self.coord_prey[1]) ** 2)
        dist_to_teammate = sqrt((self.coord_teammate[0] ** 2 + self.coord_teammate[1]) ** 2)
        distances = [dist_to_hunter, dist_to_teammate, dist_to_prey]

        # If it detects a wall_avoiding_angle it's only a "wall" when both prey, hunter and teammate centroids are (0,0)
        if dist_to_hunter != self.min_range_detected and dist_to_prey != self.min_range_detected \
                and dist_to_teammate != self.min_range_detected and self.min_range_detected < 0.8:
            self.state = 'avoid_wall'
            # print(Fore.RED + 'My name is ' + self.name + ' and I am too close to the wall. Avoiding it.' + Fore.RESET)
            # cprint("\nThank you for using AR Paint, hope to you see you again soon\n", color='white',
            #        on_color='on_blue')

        # If it detects a hunter and no prey, the player will flee away
        elif self.centroid_hunter != (0, 0) and self.centroid_prey == (0, 0) and self.centroid_teammate == (0, 0) \
                and min(distances) == dist_to_hunter:
            self.state = 'flee'

        # If it detects a prey and no hunter, the player will attack
        elif self.centroid_hunter == (0, 0) and self.centroid_prey != (0, 0) and self.centroid_teammate == (0, 0)\
                and min(distances) == dist_to_prey:
            self.state = 'attack'

        # If it detects a prey and no hunter, the player will attack
        elif self.centroid_hunter == (0, 0) and self.centroid_prey == (0, 0) and \
                self.centroid_teammate != (0, 0) and min(distances) == dist_to_teammate:
            self.state = 'avoid_wall'

        # If it detects a prey and a hunter, the player will make a decision based on the distance between both
        elif self.centroid_hunter != (0, 0) and self.centroid_prey != (0, 0) and self.centroid_teammate == (0, 0):

            # Calculates the euclidean distance between the two centroids
            self.distance_hunter_to_prey = dist_to_hunter - dist_to_prey
            # Threshold training with gazebo
            if self.distance_hunter_to_prey > 0.5:
                self.state = 'attack'
            else:
                self.state = 'flee'

        elif self.centroid_hunter != (0, 0) and self.centroid_prey != (0, 0) and self.centroid_teammate != (0, 0):

            if min(distances) == dist_to_hunter:
                self.state = 'flee'

            if min(distances) == dist_to_prey and self.distance_hunter_to_prey > 0.5:
                self.state = 'attack'

            elif min(distances) == dist_to_prey and self.distance_hunter_to_prey > 0.5:
                self.state = 'flee'

            if min(distances) == dist_to_teammate:
                self.state = 'avoid_wall'

        elif self.centroid_hunter != (0, 0) and self.centroid_prey == (0, 0) and self.centroid_teammate != (0, 0):
            if min(distances) == dist_to_hunter:
                self.state = 'flee'

            if min(distances) == dist_to_teammate:
                self.state = 'avoid_wall'

        elif self.centroid_hunter == (0, 0) and self.centroid_prey != (0, 0) and self.centroid_teammate != (0, 0):

            if min(distances) == dist_to_prey:
                self.state = 'attack'

            if min(distances) == dist_to_teammate:
                self.state = 'avoid_wall'

        # If it detects no prey, no hunter and no wall, the player will wait and walk around
        else:
            self.state = 'wait'

        # elif self.centroid_teammate != (0,0):
        #     self.state = 'avoid_teammate'

        # self.publisher_robot_state.publish(self.state)  ###

    # ------------------------------------------------------
    #               lidarScanCallback function
    # ------------------------------------------------------
    def lidarScanCallback(self, msgScan):
        """
            Create a mask with the largest blob of mask_original and return its centroid coordinates
            :param team: Team name - String
            :param image: Cv2 image - Uint8
            :return centroid: list of tuples with (x,y) coordinates of the largest object
            :return player_identified: Cv2 image mask - Uint8
        """

        # <--------------------------------Rviz visualizing part--------------------------------------------->
        header = std_msgs.msg.Header(seq=msgScan.header.seq, stamp=msgScan.header.stamp,
                                     frame_id=msgScan.header.frame_id)
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]

        # convert from polar coordinates to cartesian and fill the point cloud
        self.lidar_points = []
        z = 0
        for idx, range in enumerate(msgScan.ranges):
            theta = msgScan.angle_min + msgScan.angle_increment * idx
            x = range * math.cos(theta)
            y = range * math.sin(theta)
            self.lidar_points.append([x, y, z])


        # Call function to transform real points to pixels
        self.lidarPointsToPixels(self.lidar_points)

        real_coord_hunter, real_coord_prey, real_coord_teammate = self.find_coordinates_of_centroid()
        self.coord_hunter = (real_coord_hunter.pose.position.x, real_coord_hunter.pose.position.y)
        self.coord_prey = (real_coord_prey.pose.position.x, real_coord_prey.pose.position.y)
        self.coord_teammate = (real_coord_teammate.pose.position.x, real_coord_teammate.pose.position.y)

        # create point_cloud2 data structure
        self.pc2 = point_cloud2.create_cloud(header, fields, self.lidar_points)

        # publish (will automatically convert from point_cloud2 to Point cloud2 message)
        self.publisher_laser_distance.publish(self.pc2)

        # Shortest obstacle
        # rospy.loginfo('Find shortest obstacle')

        # <------------------------------------Gazebo part-------------------------------------------------->
        if msgScan.ranges:  # If detects something

            min_range_detected_idx = msgScan.ranges.index(min(msgScan.ranges))  # find the shortest distance index
            self.min_range_angle = msgScan.angle_min + min_range_detected_idx * msgScan.angle_increment  # min range angle
            self.min_range_detected = msgScan.ranges[min_range_detected_idx]  # min range

            # x_min_detected = round(self.min_range_detected * cos(self.min_range_angle),3)
            # y_min_detected = round(self.min_range_detected * sin(self.min_range_angle),3)

            x_min_detected = self.min_range_detected * cos(self.min_range_angle)
            y_min_detected = self.min_range_detected * sin(self.min_range_angle)
            self.min_range_point = (x_min_detected, y_min_detected, 0)

            if msgScan.ranges[min_range_detected_idx] < 1.5 and not \
                    (np.pi / 6 + np.pi / 2 <= self.min_range_angle <= 3 * np.pi / 2 - np.pi / 6):

                if self.min_range_angle < np.pi / 2:
                    self.wall_avoiding_angle = self.min_range_angle - (np.pi / 6 + np.pi / 2)

                else:
                    self.wall_avoiding_angle = self.min_range_angle - (3 * np.pi / 2 - np.pi / 6)
            else:
                self.wall_avoiding_angle = 0
        else:
            self.wall_avoiding_angle = 0

        # range_angles = np.arange(len(msgScan.ranges))
        # ranges = np.array(msgScan.ranges)
        # range_mask = (ranges > self.laser_thresh)
        # ranges = list(range_angles[range_mask])
        #
        # gap_list = []
        #
        # for k, g in groupby(enumerate(ranges), lambda ix: ix[1] - ix[0]):
        #     g = (map(itemgetter(1), g))
        #     g = list(map(int, g))
        #     gap_list.append((g[0], g[-1]))
        # gap_list.sort(key=len)
        #
        # largest_gap = gap_list[-1]
        # min_angle, max_angle = largest_gap[0] * (msgScan.angle_increment * 180 / np.pi), largest_gap[-1] * (
        #         msgScan.angle_increment * 180 / np.pi)
        # self.average_gap = (max_angle - min_angle) / 2
        #
        # self.wall_avoiding_angle = min_angle + self.average_gap

        # -------------------------------------------------------------------------------------------------------------


    # ------------------------------------------------------
    #               takeAction function
    # ------------------------------------------------------
    def takeAction(self, max_speed=1.1):
        """
           Provides action information to robot based on decisionMaking function
           :param state: Robot state - String
           :param centroid_prey: (x,y) - Tuple
           :param centroid_hunter: (x,y) - Tuple
           :param wall_avoiding_angle : angle in radians
           :return Linear velocities based on state : meters per second
           :return Angular velocities based on state: radians per second
        """

        # Nothing Detected nearby
        if self.state == 'wait':
            # if self.in_front_points != [] and all(self.in_front_points):
            #     self.linear_vel_to_wait = 0.3
            #     self.angular_vel_to_wait = 0
            # else:
            self.linear_vel_to_wait = 0.5
            self.angular_vel_to_wait = 0.3

        # Prey Detected -> Attack
        elif self.state == 'attack':

            if (self.width / 2 - self.centroid_prey[0]) < 0:
                rotation_direction = -1
                speed = self.centroid_prey[0]
            else:
                rotation_direction = 1
                speed = self.width - self.centroid_prey[0]

            angular_vel_to_attack = 0.001 * rotation_direction * speed

            # angular_vel_to_attack = 0.001 * (self.width / 2 - self.centroid_prey[0])
            # if np.sign(self.odom.twist.twist.linear.x) != np.sign(self.angular_vel_to_attack):
            #     self.angular_vel_to_attack = 2 * self.angular_vel_to_attack
            self.linear_vel_to_attack = 1.0
            self.angular_vel_to_attack = min(angular_vel_to_attack, max_speed)

            # self.linear_vel_to_attack = min(self.linear_vel_to_attack, max_attack_speed)
            # self.linear_vel_to_attack = max(self.linear_vel_to_attack, min_attack_speed)

        # Hunter Detected -> Flee
        elif self.state == 'flee':

            if (self.width / 2 - self.centroid_hunter[0]) > 0:
                rotation_direction = -1
                speed = self.centroid_hunter[0]
            else:
                rotation_direction = 1
                speed = self.width - self.centroid_hunter[0]

            angular_vel_to_flee = 0.001 * rotation_direction * speed
            # rospy.loginfo('this is the angular' + str(angular_vel_to_flee))
            self.linear_vel_to_flee = 0.8
            self.angular_vel_to_flee = min(angular_vel_to_flee, max_speed)
            # rospy.loginfo('this is the self' + str(self.angular_vel_to_flee))

        # # Teammate detected -> Avoid it
        # elif self.state == 'avoid_teammate':
        #
        #     if (self.width / 2 - self.centroid_teammate[0]) > 0:
        #         rotation_direction = -1
        #         speed = self.centroid_teammate[0]
        #     else:
        #         rotation_direction = 1
        #         speed = self.width - self.centroid_teammate[0]
        #
        #     angular_vel_to_avoid_teammate = 0.001 * rotation_direction * speed
        #
        #     self.linear_vel_to_avoid_teammate = 0.8
        #     self.angular_vel_to_avoid_teammate = min(angular_vel_to_avoid_teammate, max_speed)

        # Only Wall Detected -> Avoid_wall
        elif self.state == 'avoid_wall':

            self.angular_vel_to_avoid_wall = self.wall_avoiding_angle
            self.linear_vel_to_avoid_wall = 0.6
            #
            if self.angular_vel_to_avoid_wall < -np.pi / 4 or self.angular_vel_to_avoid_wall > np.pi / 4:
                self.linear_vel_to_avoid_wall = 0
                self.angular_vel_to_avoid_wall = 1.4
            # if self.in_front_points != [] and all(self.in_front_points):
            #     self.linear_vel_to_avoid_wall = 0.3
            #     self.angular_vel_to_avoid_wall = 0

    def printScores(self):

        print(Style.BRIGHT + '\nPlayer by player scores:' + Style.RESET_ALL)
        table = PrettyTable(
            [Back.LIGHTWHITE_EX + "Player", "Team", "State" + Style.RESET_ALL])
        if self.team == 'Red':
            player_color = Fore.RED
        elif self.team == 'Green':
            player_color = Fore.GREEN
        elif self.team == 'Blue':
            player_color = Fore.BLUE

        table.add_row([player_color + self.name + Fore.RESET,
                       player_color + self.team + Fore.RESET, self.state])

        table.align = 'c'
        table.align[Back.LIGHTWHITE_EX + "Player"] = 'l'
        table.align['Team'] = 'l'
        print(table)

    def createMarker(self, id):

        marker = Marker()
        marker.id = id
        marker.header.frame_id = self.name + '/base_scan'
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = self.color_marker1
        marker.color.g = self.color_marker0
        marker.color.b = self.color_marker0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        return marker

    def getCameraInfoCallback(self, camera):
        try:
            self.D = camera.D
            self.K = camera.K
            self.P = camera.P

            self.camera_info_exist = True

        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.camera_info_exist = False
            rospy.logerr("Error, no camera parameters")

    def lidarPointsToPixels(self, realpoints):

        # Reset the lidar printed points
        self.lidar_pixels_in_image = []
        # Camera Intrinsic matrix
        K = np.asarray(self.K).reshape((3, 3))

        # Camera Extrinsic Parameters from:
        # "rosrun tf tf_echo R1/camera_rgb_optical_frame R1/base_scan"
        RPY = (3.140, -1.122, -1.569)
        XYZ = (0.000, 0.485, 0.074)

        rotation_matrix = np.array([[np.cos(RPY[1]) * np.cos(RPY[2]),
                                     np.sin(RPY[0]) * np.sin(RPY[1]) * np.cos(RPY[2]) - np.sin(RPY[2]) * np.cos(RPY[0]),
                                     np.sin(RPY[1]) * np.cos(RPY[0]) * np.cos(RPY[2]) + np.sin(RPY[0]) * np.sin(
                                         RPY[2])],
                                    [np.sin(RPY[2]) * np.cos(RPY[1]),
                                     np.sin(RPY[0]) * np.sin(RPY[1]) * np.sin(RPY[2]) + np.cos(RPY[0]) * np.cos(RPY[2]),
                                     np.sin(RPY[1]) * np.sin(RPY[2]) * np.cos(RPY[0]) - np.sin(RPY[0]) * np.cos(
                                         RPY[2])],
                                    [-np.sin(RPY[1]), np.sin(RPY[0]) * np.cos(RPY[1]),
                                     np.cos(RPY[0]) * np.cos(RPY[1])]])
        #
        translation_matrix = np.vstack(XYZ)

        L_t_C = np.concatenate((rotation_matrix, translation_matrix), axis=1)
        rospy.loginfo(L_t_C)

        for point in realpoints:
            p = np.vstack((point[0], point[1], 0, 1))
            points_in_camera = np.dot(L_t_C, p)
            points_in_camera = np.dot(K, points_in_camera)

            if not math.isnan(points_in_camera[0]) and not math.isnan(points_in_camera[1]):
                lidar_pixel = [points_in_camera[0] / points_in_camera[2], points_in_camera[1] / points_in_camera[2],
                               points_in_camera[2]]

                if 0 < lidar_pixel[0] < self.width and 0 < lidar_pixel[1] < self.height:
                    self.lidar_pixels_in_image.append(lidar_pixel)
                self.lidar_pixels.append(lidar_pixel)
            else:
                self.lidar_pixels.append([-1000, -1000, -1000])

    def find_coordinates_of_centroid(self):

        # Create pose stamped variables
        previous_lidar_hunter_point = PoseStamped()
        previous_lidar_prey_point = PoseStamped()
        previous_lidar_teammate_point = PoseStamped()

        Closer_lidar_hunter_point = PoseStamped()
        Closer_lidar_prey_point = PoseStamped()
        Closer_lidar_teammate_point = PoseStamped()

        # Initialize variables
        Closer_lidar_hunter_point.pose.position.x = math.inf
        Closer_lidar_hunter_point.pose.position.y = math.inf

        Closer_lidar_prey_point.pose.position.x = math.inf
        Closer_lidar_prey_point.pose.position.y = math.inf

        Closer_lidar_teammate_point.pose.position.x = math.inf
        Closer_lidar_teammate_point.pose.position.y = math.inf

        previous_lidar_hunter_point.pose.position.x = math.inf
        previous_lidar_hunter_point.pose.position.y = math.inf

        previous_lidar_prey_point.pose.position.x = math.inf
        previous_lidar_prey_point.pose.position.y = math.inf

        previous_lidar_teammate_point.pose.position.x = math.inf
        previous_lidar_teammate_point.pose.position.y = math.inf

        dist_hunter_previous = 1000
        dist_prey_previous = 1000
        dist_teammate_previous = 1000
        if self.centroid_hunter != (0, 0):
            for idx_hunter, pixel_hunter in enumerate(self.lidar_pixels):
                dist_hunter = math.sqrt(
                    (pixel_hunter[0] - self.centroid_hunter[0]) ** 2 + (pixel_hunter[1] - self.centroid_hunter[1]) ** 2)
                if dist_hunter < dist_hunter_previous:
                    dist_hunter_previous = dist_hunter
                    previous_lidar_hunter_point.pose.position.x = self.lidar_points[idx_hunter][0]
                    previous_lidar_hunter_point.pose.position.y = self.lidar_points[idx_hunter][1]

        if self.centroid_prey != (0, 0):
            for idx_prey, pixel_prey in enumerate(self.lidar_pixels):
                dist_prey = math.sqrt((pixel_prey[0] - self.centroid_prey[0]) ** 2 + (
                        pixel_prey[1] - self.centroid_prey[1]) ** 2)
                if dist_prey < dist_prey_previous:
                    dist_prey_previous = dist_prey
                    previous_lidar_prey_point.pose.position.x = self.lidar_points[idx_prey][0]
                    previous_lidar_prey_point.pose.position.y = self.lidar_points[idx_prey][1]

        if self.centroid_teammate != (0, 0):
            for idx_teammate, pixel_teammate in enumerate(self.lidar_pixels):
                # rospy.loginfo('im here')
                # rospy.loginfo(len(self.lidar_pixels))
                dist_teammate = math.sqrt((pixel_teammate[0] - self.centroid_teammate[0]) ** 2 + (
                        pixel_teammate[1] - self.centroid_teammate[1]) ** 2)
                if dist_teammate < dist_teammate_previous:
                    dist_teammate_previous = dist_teammate
                    previous_lidar_teammate_point.pose.position.x = self.lidar_points[idx_teammate][0]
                    previous_lidar_teammate_point.pose.position.y = self.lidar_points[idx_teammate][1]
        self.lidar_pixels = []
        Closer_lidar_hunter_point.pose.position.x = previous_lidar_hunter_point.pose.position.x
        Closer_lidar_hunter_point.pose.position.y = previous_lidar_hunter_point.pose.position.y

        Closer_lidar_prey_point.pose.position.x = previous_lidar_prey_point.pose.position.x
        Closer_lidar_prey_point.pose.position.y = previous_lidar_prey_point.pose.position.y

        Closer_lidar_teammate_point.pose.position.x = previous_lidar_teammate_point.pose.position.x
        Closer_lidar_teammate_point.pose.position.y = previous_lidar_teammate_point.pose.position.y

        return Closer_lidar_hunter_point, Closer_lidar_prey_point, Closer_lidar_teammate_point

    # def chatting(self):
    #     self.actual_state = self.state
    #     if self.actual_state != self.old_state: # if state changed -> print state msg
    #         if self.state.__eq__('wait'):
    #             self.state_msg = 'What a deadly bore'
    #         elif self.state.__eq__('attack'):
    #             self.state_msg = 'I am very hungry'
    #         elif self.state.__eq__('flee'):
    #             self.state_msg = 'Oh no, I have to run'
    #         else:
    #             self.state_msg = 'Walls everywhere in this game'
    #         # print(self.name + self.state_msg) # print state msg
    #         self.robot_state_message = self.name + ': ' + self.state_msg
    #         self.robot_state_publisher.publish(self.robot_state_message)
    #     self.old_state = self.actual_state

    def chatting(self):
        self.actual_state = self.state
        if self.actual_state != self.old_state:
            if self.state.__eq__('wait'):
                waiting_list = ['I really need a beer right now', 'This game is so boring', 'I have no time to lose...']
                self.state_msg = random.choice(waiting_list)
            elif self.state.__eq__('attack'):
                attack_list = ['You better be scared', 'Eat my shorts', 'Run to the hills']
                self.state_msg = random.choice(attack_list) + ', ' + self.prey.lower()
            elif self.state.__eq__('flee'):
                # self.state_msg = 'Oh no, I have to run! ' + self.hunter.lower() + ' is coming'
                fleeing_list = ['Oh no, I have to run from ', 'You are a snail ', 'You look like my grandma ']
                self.state_msg = random.choice(fleeing_list) + self.hunter.lower()
            else:
                avoidance_list = ['So annoying, walls everywhere in this game :(',
                                  'This place looks like the minotaur labyrinth', 'Another wall? What the hell?']
                self.state_msg = random.choice(avoidance_list)
            self.robot_state_message = self.name + ': ' + self.state_msg
            self.robot_state_publisher.publish(self.robot_state_message)
        self.old_state = self.actual_state

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
        driver.chatting()
        rate.sleep()


if __name__ == '__main__':
    main()
