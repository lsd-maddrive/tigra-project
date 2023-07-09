#!/usr/bin/env bash

cd src/maddrive_ros_shared/third_party/Lslidar_ROS1_driver
git checkout C16_V2.6/2.8/3.0
cd -

export CMAKE_PREFIX_PATH=/usr/local/lib/cmake:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

catkin clean lslidar_driver
catkin build \
    tigra-project \
    ackermann_raw_controller_plugin \
    elp_stereo_camera \
    rtabmap \
    rtabmap_ros \
    ublox \
    cv_bridge \
    stereo_image_proc \
    camera_calibration \
    maddrive_urdf_tools \
    maddrive_teleop \
    maddrive_worlds \
    lslidar_driver \
    realsense2_camera \
    realsense2_description \
    -j$(($(nproc)-2)) \
    --cmake-args -D OpenCV_DIR="/usr/local/lib/cmake/opencv4" -D RTABMAP_SYNC_MULTI_RGBD=ON -D CATKIN_ENABLE_TESTING=False -D CMAKE_BUILD_TYPE=Release
    
    # serial \
    # ti_mmwave_rospkg \
    # viso2 \
    # orb_slam2_ros \
