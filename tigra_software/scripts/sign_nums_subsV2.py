#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

def sign_detected_callback(msg):
    if msg.data:  # Если значение true
        # Отправляем нулевое управление
        zero_twist = Twist()
        cmd_vel_pub.publish(zero_twist)
    else:
        # Пропускаем управление на выход
        cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('logic_node')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('detectTrafficLights', Bool, sign_detected_callback)

    rospy.spin()