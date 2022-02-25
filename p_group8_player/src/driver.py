#!/usr/bin/python3

import copy
import rospy
import std_msgs
from std_msgs.msg import String
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from driver_functions import *
import numpy as np
import math
from math import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Image, LaserScan
from sensor_msgs import point_cloud2


class Driver():
    def __init__(self):

        # <---------------------------------------------------------------------------------------------------------->
        # <--------------------------------Variable Initialization--------------------------------------------------->
        # <---------------------------------------------------------------------------------------------------------->
        self.team = 'Not defined'
        self.prey = 'Not defined'
        self.hunter = 'Not defined'
        self.team_players = 'Not defined'
        self.hunter_team_players = 'Not defined'
        self.prey_team_players = 'Not defined'
        self.cv_image = []
        self.centroid_hunter = (0, 0)
        self.centroid_prey = (0, 0)
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
        self.min_range_detected = 0
        self.height = 0
        self.width = 0
        self.odom = None
        self.position = (0, 0)

        # Getting parameters
        red_players_list = rospy.get_param('/red_players')
        green_players_list = rospy.get_param('/green_players')
        blue_players_list = rospy.get_param('/blue_players')
        self.debug = rospy.get_param('~debug')

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

        # <---------------------------------------------------------------------------------------------------------->
        # <-----------------------------Publishers and Subscribers--------------------------------------------------->
        # <---------------------------------------------------------------------------------------------------------->
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)
        self.camera_subscriber = rospy.Subscriber('/' + self.name + '/camera/rgb/image_raw', Image, self.cameraCallback)
        self.laser_subscriber = rospy.Subscriber('/' + self.name + '/scan', LaserScan, self.lidarScanCallback)
        self.odom_subscriber = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odomPositionCallback)

        # publishing the robot state: 'wait', 'attack', 'flee' and 'avoid_wall'
        self.publisher_robot_state = rospy.Publisher('/' + self.name + '/state', String)

        self.publisher_laser_distance = rospy.Publisher('/' + self.name + '/point_cloud', PointCloud2)
        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist)
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
        elif self.goal_active:
            twist.linear.x = speed
            twist.angular.z = angle

        self.publisher_command.publish(twist)

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

        self.decisionMaking()

        if self.debug:
            # rospy.loginfo(self.distance_hunter_to_prey)
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
            cv2.putText(hunters_n_preys, str(self.state), (0, self.height - 50),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0), thickness=5)

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
        largest_object = np.zeros([self.height, self.width], np.uint8)

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
            self.perWidht = stats[largest_object_idx, cv2.CC_STAT_WIDTH]

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

        # If it detects a wall_avoiding_angle it's only a "wall" when both prey and hunter centroids are (0,0)
        if self.centroid_prey == (0, 0) and self.centroid_hunter == (0, 0) and self.wall_avoiding_angle != 0:
            self.state = 'avoid_wall'

        # If it detects a hunter and no prey, the player will flee away
        if self.centroid_hunter != (0, 0) and self.centroid_prey == (0, 0):
            self.state = 'flee'

        # If it detects a prey and no hunter, the player will attack
        if self.centroid_hunter == (0, 0) and self.centroid_prey != (0, 0):
            self.state = 'attack'

        # If it detects a prey and a hunter, the player will make a decision based on the distance between both
        if self.centroid_hunter != (0, 0) and self.centroid_prey != (0, 0):

            # Calculates the euclidean distance between the two centroids
            self.distance_hunter_to_prey = sqrt((self.centroid_hunter[0] - self.centroid_prey[0]) ** 2
                                                + (self.centroid_hunter[1] - self.centroid_prey[1]) ** 2)
            # Threshold training with gazebo
            if self.distance_hunter_to_prey > 400:
                self.state = 'attack'
            else:
                self.state = 'flee'

        # If it detects no prey, no hunter and no wall, the player will wait and walk around
        if self.centroid_hunter == (0, 0) and self.centroid_prey == (0, 0) and self.wall_avoiding_angle == 0:
            self.state = 'wait'

        self.publisher_robot_state.publish(self.state) ###

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
        points = []
        dist_angle = []
        for idx, range in enumerate(msgScan.ranges):
            theta = msgScan.angle_min + msgScan.angle_increment * idx
            x = range * math.cos(theta)
            y = range * math.sin(theta)
            points.append([x, y, 0])

        # create point_cloud2 data structure
        pc2 = point_cloud2.create_cloud(header, fields, points)

        # publish (will automatically convert from point_cloud2 to Point cloud2 message)
        # self.publisher_laser_distance.publish(pc2)

        # Shortest obstacle
        # rospy.loginfo('Find shortest obstacle')

        # <------------------------------------Gazebo part-------------------------------------------------->
        if msgScan.ranges:  # If detects something
            min_range_detected_idx = msgScan.ranges.index(min(msgScan.ranges))  # find the shortest distance index
            min_range_angle = msgScan.angle_min + min_range_detected_idx * msgScan.angle_increment  # min range angle
            self.min_range_detected = msgScan.ranges[min_range_detected_idx]

            if msgScan.ranges[min_range_detected_idx] < 3 and not \
                    (np.pi / 6 + np.pi / 2 <= min_range_angle <= 3 * np.pi / 2 - np.pi / 6):

                if min_range_angle < np.pi / 2:
                    self.wall_avoiding_angle = min_range_angle - (np.pi / 6 + np.pi / 2)

                else:
                    self.wall_avoiding_angle = min_range_angle - (3 * np.pi / 2 - np.pi / 6)
            else:
                self.wall_avoiding_angle = 0
        else:
            self.wall_avoiding_angle = 0

        # rospy.loginfo('closest object angle: ' + str(self.wall_avoiding_angle))
    # ------------------------------------------------------
    #               takeAction function
    # ------------------------------------------------------
    def takeAction(self, max_speed=1):
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
            self.linear_vel_to_wait = 0.5
            self.angular_vel_to_wait = 0.3

        # Prey Detected -> Attack
        elif self.state == 'attack':

            if (self.width / 2 - self.centroid_hunter[0]) < 0:
                rotation_direction = -1
                speed = self.centroid_hunter[0]
            else:
                rotation_direction = 1
                speed = self.width - self.centroid_hunter[0]

            angular_vel_to_attack = 0.01 * rotation_direction * speed

            # if np.sign(self.odom.twist.twist.linear.x) != np.sign(self.angular_vel_to_attack):
            #     self.angular_vel_to_attack = 2 * self.angular_vel_to_attack
            self.linear_vel_to_attack = 0.7
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
            self.linear_vel_to_flee = 0.7
            self.angular_vel_to_flee = min(angular_vel_to_flee, max_speed)
            # rospy.loginfo('this is the self' + str(self.angular_vel_to_flee))

        # Only Wall Detected -> Avoid_wall
        elif self.state == 'avoid_wall':
            self.angular_vel_to_avoid_wall = self.wall_avoiding_angle
            self.linear_vel_to_avoid_wall = 0.15
            if self.angular_vel_to_avoid_wall < -np.pi / 4 or self.angular_vel_to_avoid_wall > np.pi / 4:
                self.linear_vel_to_avoid_wall = 0


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
