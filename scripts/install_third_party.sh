#!/usr/bin/env bash

THIRD_PARTY_DIR=third_party
mkdir -p $THIRD_PARTY_DIR

# GPS driver

git -C $THIRD_PARTY_DIR clone  https://github.com/KumarRobotics/ublox -b 1.4.1
git -C $THIRD_PARTY_DIR/ublox apply ../patches/ublox.patch

# ELP stereocamera driver
#   NB - version not set as driver is under our development

git -C $THIRD_PARTY_DIR clone https://github.com/lsd-maddrive/elp_stereo_camera

# Libviso2

git -C $THIRD_PARTY_DIR clone https://github.com/srv/viso2 -b melodic_develop_sift
git -C $THIRD_PARTY_DIR/viso2 apply ../patches/viso2.patch

# ORB-SLAM2

git -C $THIRD_PARTY_DIR clone https://github.com/appliedAI-Initiative/orb_slam_2_ros -b v1.2

git -C $THIRD_PARTY_DIR clone https://github.com/ros-drivers/rosserial.git -b noetic-devel

# Rtabmap

git -C $THIRD_PARTY_DIR clone https://github.com/introlab/rtabmap_ros.git -b 0.20.10-noetic

# cv_bridge

# git -C $THIRD_PARTY_DIR clone https://github.com/ros-perception/vision_opencv.git -b noetic
