#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Int8, Bool

def sign_detected_callback(data):
    if data.data == 1:
        # output_pub.publish(True)
        # rospy.loginfo('red, stop stop stop!')
        pass

    elif data.data == 0:
        output_pub.publish(False)
        rospy.loginfo('green, go go go!')

def sign_detection_node():
    rospy.init_node('TrafficLights2Cmd', anonymous=False)
    rospy.Subscriber('detectTrafficLights', Int8, sign_detected_callback)
    global output_pub
    output_pub = rospy.Publisher('detectTrafficLightsCmd', Bool, queue_size=10)
    time.sleep(1)
    init_time = time.time()
    timeout = 30
    output_pub.publish(True)
    rospy.loginfo('trafic light is ready!')
    # rospy.spin()
    while not rospy.is_shutdown():
        if time.time() - init_time > timeout:
            rospy.loginfo('timeout!')
            output_pub.publish(False)

if __name__ == '__main__':
    sign_detection_node()