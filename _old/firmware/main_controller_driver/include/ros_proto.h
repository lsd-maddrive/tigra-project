#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <common.h>

/*** ROS prototypes ***/

/**
 * @brief   Init ROS parameters
 * @param   prio    Priority of thread
 */
void rosInit( tprio_t prio );

#ifdef __cplusplus
}
#endif
