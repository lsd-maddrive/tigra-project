#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/:$CMAKE_PREFIX_PATH

catkin build \
    tigra_msgs \
    tigra_description \
    tigra_software \
    tigra_vision \
    tigra_maps \
    ackermann_raw_controller_plugin \
    elp_stereo_camera \
    rtabmap \
    rtabmap_ros \
    spatio_temporal_voxel_layer \
    ublox \
    --cmake-args -D RTABMAP_SYNC_MULTI_RGBD=ON

    # stereo_image_proc \
    # cv_bridge \
    # camera_calibration \
    
    # serial \
    # ti_mmwave_rospkg \
    # viso2 \
    # orb_slam2_ros \
