import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import numpy as np
import threading

rospy.init_node('control_link')

steer_value = 0
speed_value = 0
current_mode = 0


def joy_cb(msg):
    global steer_value, speed_value, current_mode

    # print(msg)

    if msg.buttons[0]:
        print('X (blue) pressed')
        current_mode = 0
        mode_pub.publish( current_mode ) 

    if msg.buttons[1]:
        print('A (green) pressed')

    if msg.buttons[6] and msg.buttons[7]:
        if msg.buttons[2]:
            print('B (red) pressed')
            # Start
            current_mode = 2
            mode_pub.publish( current_mode ) 


        if msg.buttons[3]:
            print('Y (yellow) pressed')
            # Prepare
            current_mode = 1
            mode_pub.publish( current_mode ) 


    if msg.buttons[4]:
        print('LB pressed')
        steer_value = 0

    if msg.axes[0] != 0:
        steer_value += -10 * msg.axes[0]

    speed_value = msg.axes[3] * 100

    speed_limit = 100
    steer_limit = 80

    speed_value = np.clip( speed_value, -10, speed_limit ) 
    steer_value = np.clip( steer_value, -steer_limit, steer_limit ) 

    # speed_pub.publish( msg.linear.x * 100 )
    # steer_pub.publish( msg.angular.z * -150 )

    # cmd = [msg.linear.x * 100, msg.angular.z * -150]

    print('Cmd: ', [speed_value, steer_value])


rospy.Subscriber('joy', Joy, joy_cb, queue_size = 1)

speed_pub = rospy.Publisher('quadro/speed_perc', Int8, queue_size=1)
steer_pub = rospy.Publisher('quadro/steer_perc', Int8, queue_size=1)
mode_pub = rospy.Publisher('quadro/mode_status', UInt8, queue_size=1)

if __name__ == '__main__':

    print('Ready, go')

    while not rospy.is_shutdown():

        steer_pub.publish( steer_value )
        speed_pub.publish( speed_value )

        sleep( 100 / 1000. )
