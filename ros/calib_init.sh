#!/bin/bash

roslaunch cameras_calib.launch camera_info:=true \
			video_device_right:=/dev/video1 \
			video_device_left:=/dev/video0

