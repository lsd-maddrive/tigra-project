#!/usr/bin/env bash

sudo apt-get install \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rosserial-server \
    ros-$ROS_DISTRO-rosserial-client \
    ros-$ROS_DISTRO-rtabmap-ros \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-global-planner \
    ros-$ROS_DISTRO-teb-local-planner
