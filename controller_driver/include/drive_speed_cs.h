#ifndef INCLUDE_DRIVE_SPEED_CS_H_
#define INCLUDE_DRIVE_SPEED_CS_H_

#include <common.h>
#include <lld_wheel_pos_sensor.h>
#include <lld_control.h>

/***Hardware description***/


/*** Variables ***/



/*** Prototypes ***/

/*
 *@brief         Low level drivers initialization
 *@note          First time function call sets flag "isInitialized"
 *               which protects of multiple initialization
 */
void DriveSpeedCSInit( void );

/*
 * @brief        Control system "shell".
 *               Function calculates the difference between reference and
 *               current speed. Calls the controller function which calculates
 *               control action value. Sets the power value (and DIRECION?)
 *               to the motor.
 * @params [in]  Speed reference value
 * @note         If speed reference value doesn't match the limits
 *               it will be saturated
 * @return       Controller output, if all required lld's is initialized
 *               -1               , if not
 */
int32_t DriveSpeedControl ( wheelVelocity_t speedReference );

#endif /* INCLUDE_DRIVE_SPEED_CS_H_*/
