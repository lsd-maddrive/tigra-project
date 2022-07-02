#!/usr/bin/env python
# TODO - not completed
import os
import rospy
import yaml
import copy
from sensor_msgs.msg import CameraInfo
import rospkg
import sys


def callback(msg):
    camera_info_msg.header = msg.header
    publisher.publish(camera_info_msg)
    # rospy.loginfo("I heard %s", msg.data)

class CameraInfoRepublisher:
    def __init__(calib_fpath, topic) -> None:
        self._calib_fpath = calib_fpath
        self._topic = topic

        self._camera_info_msg_base = self._generate_camera_info(self._calib_fpath)

        self._sub = rospy.Subscriber(self._topic, CameraInfo, callback, queue_size=10)


    def _generate_camera_info(self, fpath):
        with open(fpath, "r") as file_handle:
            calib_data = yaml.load(file_handle)

        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["camera_model"]
        return camera_info_msg

    def _callback(self, msg):
        camera_info = copy.deepcopy(self._camera_info_msg_base)
        

if __name__ == "__main__":

    rospy.init_node("camera_info_publisher", anonymous=True)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('tigra_vision')

    left_repub = CameraInfoRepublisher(calib_fpath=os.path.join(pkg_path, "calib_info", "xiaomi_wooden_stereo", 'left.yaml'), )
    right_repub = CameraInfoRepublisher(calib_fpath=os.path.join(pkg_path, "calib_info", "xiaomi_wooden_stereo", 'right.yaml'))


    direction = rospy.get_param('~direction')
    # file_name = rospy.get_param('~file_name')
    file_name = direction + '.yaml'
    


    # Initialize publisher node
    topic_name = "/stereo/from_param/" + direction + "_camera_info"
    publisher = rospy.Publisher(topic_name, CameraInfo, queue_size=10)
    # rate = rospy.Rate(30)

    # Run publisher
    while not rospy.is_shutdown():
        # publisher.publish(camera_info_msg)
        # rate.sleep()