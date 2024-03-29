#ifndef INCLUDE_DRIVE_SPEED_CS_H_
#define INCLUDE_DRIVE_SPEED_CS_H_

#include <common.h>
#include <lld_wheel_pos_sensor.h>
#include <lld_control.h>
#include <light_unit.h>

#include <controllers.h>

/***Hardware description***/


/*** Variables ***/


/*** Prototypes ***/

/*
 *@brief         Low level drivers initialization
 *@note          First time function call sets flag "isInitialized"
 *               which protects of multiple initialization
 */
void driveSpeedCSInit( void );

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
 *          TODO check comments
 */
controlValue_t driveSpeedControl ( int32_t speedReference );

bool isForward( controlValue_t motorPower );

#endif /* INCLUDE_DRIVE_SPEED_CS_H_*/
