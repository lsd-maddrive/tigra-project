#!/usr/bin/env bash

catkin build \
    tigra-project \
    maddrive_ros_shared \
    -j$(($(nproc)-2))

