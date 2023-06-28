#!/usr/bin/env bash

sudo apt-get install \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rosserial-server \
    ros-$ROS_DISTRO-rosserial-client \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-global-planner \
    ros-$ROS_DISTRO-teb-local-planner \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-hector-gazebo-plugins \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    libopenvdb-dev \
    libpcap-dev

#requirements for RS-ros
sudo apt purge ros-$ROS_DISTRO-realsense2-camera