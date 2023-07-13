#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Int8, Bool

def sign_detected_callback(data):
    if data.data == 1:
        time.sleep(0.2)
        if data.data == 1:
            output_pub.publish(True)

def sign_detection_node():
    rospy.init_node('TrafficLights2Cmd', anonymous=False)
    rospy.Subscriber('detectTrafficLights', Int8, sign_detected_callback)
    global output_pub
    output_pub = rospy.Publisher('detectTrafficLightsCmd', Bool, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    sign_detection_node()