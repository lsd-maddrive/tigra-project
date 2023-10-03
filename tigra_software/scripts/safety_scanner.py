#!/usr/bin/env python3

import rospy
import actionlib
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
scann = LaserScan()
mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
safe_range = 1.5


def callback(msg):
    #print(len(msg.ranges)) len is 2019 from 0-360
    # current_time = rospy.Time.now()
    # scann.header.stamp = current_time
    # scann.header.frame_id = 'laser'
    # scann.angle_min = -3.1415
    # scann.angle_max = 3.1415
    # scann.angle_increment = 0.00311202858575
    # scann.time_increment = 4.99999987369e-05
    # scann.range_min = 0.00999999977648
    # scann.range_max = 32.0
    # scann.ranges = msg.ranges[0:72]
    # scann.intensities = msg.intensities[0:72]
    # print(scann)
    # pub.publish(scann)
    for x in msg.ranges:
        if x < safe_range:
            rospy.loginfo(f'A scanner stop occurred at range {x}')
            mb_client.cancel_goal()
            mb_client.cancel_all_goals() # Try both
            rospy.loginfo("Goal cancelled")

def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/lslidar_safe_scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()