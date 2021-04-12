#!/usr/bin/env python
import rospy
import struct
# import plc
import ros_numpy
import message_filters
import numpy

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

data = 0
pub = rospy.Publisher('/circle_overview_bot/concatenate_data', PointCloud2, queue_size=10)

def callback(front_side_data, right_side_data):


    xyz_array_right = ros_numpy.point_cloud2.pointcloud2_to_array(right_side_data)
    xyz_array_front = ros_numpy.point_cloud2.pointcloud2_to_array(front_side_data)

    concatenate_value = numpy.concatenate((xyz_array_front, xyz_array_right), axis=1)

    print(concatenate_value.shape)
    
    ready_msg = ros_numpy.point_cloud2.array_to_pointcloud2(concatenate_value, frame_id='rightside_left_optical_frame')
    

    print(ready_msg.width)
    pub.publish(ready_msg)
    print("I'm fine")




rospy.init_node('listener')

front_side_data =message_filters.Subscriber('/circle_overview_bot/front/points2', PointCloud2)
right_side_data = message_filters.Subscriber('/circle_overview_bot/rightside/points2', PointCloud2)

ts = message_filters.TimeSynchronizer([front_side_data, right_side_data], 10)
ts.registerCallback(callback)
rospy.spin()


