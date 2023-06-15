#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/:$CMAKE_PREFIX_PATH
# export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH

catkin build \
    tigra-project \
    tigra_vision \
    ackermann_raw_controller_plugin \
    ublox \
    rtabmap \
    rtabmap_ros \
    spatio_temporal_voxel_layer \
    camera_calibration \
    elp_stereo_camera \
    stereo_image_proc \
    cv_bridge \
    maddrive_urdf_tools \
    lslidar_c16 \
    lslidar_driver \
    -j$(($(nproc)-2)) \
    --cmake-args -D OpenCV_DIR="/usr/local/lib/cmake/opencv4" -D RTABMAP_SYNC_MULTI_RGBD=ON -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
    
    # serial \
    # ti_mmwave_rospkg \
    # viso2 \
    # orb_slam2_ros \
