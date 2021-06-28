#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import numpy as np


# 0 - Blue
# 1 - Green
# 2 - Red
# 3 - Yellow
# 4 - LU
# 5 - RU
# 6 - LB
# 7 - RB
# 10 - Left stick
# 11 - Right stick

button_names = [
    "Blue",
    "Green",
    "Red",
    "Yellow",
    "Left upper",
    "Right upper",
    "Left bottom",
    "Right bottom",
    "_",
    "_",
    "Left stick",
    "Right stick",
]

axes_names = [
    "Lstick horz",
    "Lstick vert",
    "Rstick horz",
    "Rstick vert",
    "Btns horz",
    "Btns vert",
]


def show_clicked(msg):
    print("Buttons:")
    for i in range(len(msg.buttons)):
        if msg.buttons[i] != 0:
            print("\t" + button_names[i])

    print("Axes:")
    for i in range(len(msg.axes)):
        print("\t%s: %.2f" % (axes_names[i], msg.axes[i]))


def joy_cb(msg):
    global steer_value, speed_value, current_mode

    if debug_enabled:
        show_clicked(msg)

    # if msg.buttons[0]:
    #     current_mode = 0
    #     mode_pub.publish(current_mode)

    if msg.buttons[1]:
        pass

    # Press yellow then red

    # if msg.buttons[6] and msg.buttons[7]:
    #     if msg.buttons[3]:
    #         # Prepare
    #         current_mode = 1
    #         mode_pub.publish(current_mode)

    #     if msg.buttons[2]:
    #         # Start
    #         current_mode = 2
    #         mode_pub.publish(current_mode)

    if msg.buttons[4]:
        steer_value = 0

    # if msg.axes[0] != 0:
    #     steer_value += -10 * msg.axes[0]

    steer_value = msg.axes[0] * 80
    speed_value = msg.axes[3] * 100

    speed_limit = 100
    steer_limit = 80

    if speed_value < 0:
        speed_value = speed_value * 0.3

    speed_value = np.clip(speed_value, -30, speed_limit)
    steer_value = np.clip(steer_value, -steer_limit, steer_limit)

    # speed_pub.publish( msg.linear.x * 100 )
    # steer_pub.publish( msg.angular.z * -150 )

    # cmd = [msg.linear.x * 100, msg.angular.z * -150]

    # print('Cmd: ', [speed_value, steer_value])


if __name__ == "__main__":
    rospy.init_node("control_link")

    debug_enabled = rospy.get_param("~debug", False)
    if debug_enabled:
        print("Debug enabled")
    
    command_state = Twist()

    rospy.Subscriber("joy", Joy, joy_cb, queue_size=5)
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    speed_limit_mps = rospy.get_param("~speed_limit", 5)
    steer_limit_deg = rospy.get_param("~steer_limit", 25)
    steer_limit_rad = np.deg2rad(steer_limit_deg)

    rospy.logdebug("Ready, go!")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        cmd_pub.publish(command_state)
        rate.sleep()
