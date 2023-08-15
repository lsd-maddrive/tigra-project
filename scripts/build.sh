#!/usr/bin/env bash

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

cd src/maddrive_ros_shared/third_party/Lslidar_ROS1_driver
git checkout C16_V2.6/2.8/3.0
cd -

if catkin list | grep -q "lslidar_driver"; then
    catkin clean lslidar_driver
fi

catkin build \
    tigra-project \
    maddrive_ros_shared \
    ackermann_raw_controller_plugin \
    elp_stereo_camera \
    rtabmap \
    rtabmap_ros \
    ublox \
    cv_bridge \
    stereo_image_proc \
    camera_calibration \
    lslidar_driver \
    realsense2_camera \
    realsense2_description \
    image_geometry \
    hector_gazebo \
    point_cloud_converter \
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
