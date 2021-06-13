#ifndef ROS_H
#define ROS_H

#include <ros/node_handle.h>
#include <socket_hardware.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

namespace ros
{
    typedef NodeHandle_<SocketHardware, 10, 10, 512, 512> NodeHandle; // default 25, 25, 512, 512
}

// #ifdef __cplusplus
// }
// #endif

#endif /* ROS_H */
