#pragma once

#include <common.h>

#ifdef __cplusplus
extern "C" {
#endif

/*** ROS prototypes ***/

/**
 * @brief   Init ROS parameters
 * @param   prio    Priority of thread
 */
void rosInit( tprio_t prio );

/**
 * @brief           Send test int32 message
 * @param   value   int32 value to send
 */
void rosSendTestI32Msg( int32_t value );

/**
 * @brief           Set callback of service server
 * @param   cb_func Callback function
 */
void rosTestSrvSetCb( bool (*cb_func)( void ) );

#ifdef __cplusplus
}
#endif
