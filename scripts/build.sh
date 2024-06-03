#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

catkin build \
    tigra-project \
    maddrive_ros_shared \
    rtabmap \
    rtabmap_ros \
    ublox \
    cv_bridge \
    stereo_image_proc \
    camera_calibration \
    lslidar_c16_driver \
    realsense2_camera \
    realsense2_description \
    image_geometry \
    hector_gazebo \
    velodyne_simulator \
    ublox \
    rtcm_msgs \
    nmea_msgs \
    ntrip_client \
    mavros \
    rviz_satellite \
    -j$(($(nproc)-2)) \
    --cmake-args \
    -D OpenCV_DIR="/usr/local/lib/cmake/opencv4" \
    -D RTABMAP_SYNC_MULTI_RGBD=ON \
    -D CATKIN_ENABLE_TESTING=False \
    -D CMAKE_BUILD_TYPE=Release
    
    # serial \
    # ti_mmwave_rospkg \
    # viso2 \
    # orb_slam2_ros \
