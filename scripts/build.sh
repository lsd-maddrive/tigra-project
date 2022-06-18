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
    cv_bridge \
    rtabmap \
    rtabmap_ros \
    stereo_image_proc \
    spatio_temporal_voxel_layer \
    camera_calibration \
    --cmake-args -D OpenCV_DIR="/usr/local/lib/cmake/opencv4" \
                 -D RTABMAP_SYNC_MULTI_RGBD=ON

    # ublox \
    # serial \
    # ti_mmwave_rospkg \
    # viso2 \
    # orb_slam2_ros \
