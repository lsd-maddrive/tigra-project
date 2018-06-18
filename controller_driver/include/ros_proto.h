#pragma once

#include <common.h>

#ifdef __cplusplus
extern "C" {
#endif

/*** ROS prototypes ***/

/**
 * @brief   Init ROS parameters
 * @note    Call it first
 */
void ros_driver_init( void );

/**
 * @brief           Start ROS thread
 * @param   prio    Priority of thread
 */
void ros_driver_start( tprio_t prio );

/**
 * @brief           Send test int32 message
 * @param   value   int32 value to send
 */
void ros_send_test_i32_msg( int32_t value );

/**
 * @brief           Set callback of service server
 * @param   cb_func Callback function
 */
void ros_test_srv_set_cb( bool (*cb_func)( int32_t value ) );

#ifdef __cplusplus
}
#endif
