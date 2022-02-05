#!/usr/bin/env python3

# Imports
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# Direction Callback Function
def messageReceivedCallbackJoy(message):
    global twist_publisher, velocity, velocity_temp
    rospy.loginfo(f'Joy message received: \n{message}')

    stop_bool = bool(message.buttons[0])
    velocity_bool = bool(message.buttons[1])
    add_velocity_bool = bool(message.buttons[2])
    remove_velocity_bool = bool(message.buttons[3])
    boost_trigger_bool = bool(message.buttons[8])
    # If right button is pressed, velocity is one
    if velocity_bool:
        velocity = 1
    # If bottom button is pressed, velocity is zero
    elif stop_bool:
        velocity = 0
    # If top button is pressed, velocity is increased
    elif add_velocity_bool:
        velocity += 0.1
    # If left button is pressed, velocity is decreased
    elif remove_velocity_bool:
        velocity -= 0.1
    # If the right trigger is pressed, the velocity is temporarily superior
    elif boost_trigger_bool:
        velocity_temp = velocity
        velocity += 0.5

    # Defining the angular velocity from the joystick values
    angular_float = message.axes[0]
    twist = Twist()

    # Defining and publishing the twist
    twist.linear.x = velocity
    twist.angular.z = angular_float
    twist_publisher.publish(twist)
    rospy.loginfo(f'Sent twist: \n{twist}')

    # Restoring original velocity
    if velocity_temp:
        velocity = velocity_temp
        velocity_temp = None


def main():
    global twist_publisher, velocity, velocity_temp

    # Initiating node
    rospy.init_node('joystick_to_cmd_vel', anonymous=False)

    # Get parameters
    twist_topic = rospy.get_param('~twist_dir_topic', '/p_randomName/cmd_vel')
    joy_topic = rospy.get_param('~joy_topic', '/joy')

    # Define variables, publishers and subscribers
    velocity = 0
    velocity_temp = None
    twist_publisher = rospy.Publisher(twist_topic, Twist, queue_size=10)
    rospy.Subscriber(joy_topic, Joy, messageReceivedCallbackJoy)
    rospy.spin()

if __name__ == '__main__':
    main()





