#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/:$CMAKE_PREFIX_PATH
# export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH

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
    cv_bridge \
    stereo_image_proc \
    camera_calibration \
    maddrive_urdf_tools \
    --cmake-args -D OpenCV_DIR="/usr/local/lib/cmake/opencv4" -D RTABMAP_SYNC_MULTI_RGBD=ON
    
    # serial \
    # ti_mmwave_rospkg \
    # viso2 \
    # orb_slam2_ros \
