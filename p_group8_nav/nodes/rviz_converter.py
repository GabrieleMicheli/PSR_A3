#!/usr/bin/env python3

# Imports
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def poseEstimateCallback(message):
    global pose_estimate_publisher
    rospy.loginfo('Received initial pose from RViZ')
    pose_estimate_publisher.publish(message)

def goalCallback(message):
    global goal_publisher
    rospy.loginfo('Received goal from RViZ')
    goal_publisher.publish(message)


def main():
    global pose_estimate_publisher, goal_publisher

    # Initiating node
    rospy.init_node('rviz_converter', anonymous=False)

    # Get parameters
    player_name = rospy.get_param('~player_name', 'R3')

    # Define variables, publishers and subscribers
    pose_estimate_publisher = rospy.Publisher('/' + player_name + '/initialpose', PoseWithCovarianceStamped, queue_size=10)
    goal_publisher = rospy.Publisher('/' + player_name + '/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, poseEstimateCallback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalCallback)
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    main()






