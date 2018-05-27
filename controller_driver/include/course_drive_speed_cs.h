#ifndef INCLUDE_COURSE_DRIVE_SPEED_CS_H_
#define INCLUDE_COURSE_DRIVE_SPEED_CS_H_

#include <common.h>
#include <lld_wheel_pos_sensor.h>

/***Hardware description***/


/*** Variables ***/

typedef float         coeffPIDController;

/* Configuration - PID controller parameters*/
typedef struct {

  coeffPIDController    kp;
  coeffPIDController    ki;
  coeffPIDController    kd;

}controllerParams;

/* Switching context - input parameter to controller function */
typedef struct {

    wheelVelocity_t       err;
    controllerParams      *params;

}controllerContext;


/*** Prototypes ***/



/*
 *@brief         Low level drivers initialization
 *@note          First time function call sets flag "isInitialized"
 *               which protects of multiple initialization
 */
void CourseDriveSpeedCSInit( void );


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
uint8_t  CourseDriveSpeedControl (wheelVelocity_t speedReference );


/*
 *  @brief       Control system law realization. PID controller *
 *  @params[in]  Structure, contains current error, PID controller parameters
 *  @params[out] Controller output [0;100] %
 *  @note        Access parameters like this:
 *                                            PIDContext->params->kp
 *                                            PIDContext->err
 */
uint8_t PIDController (controllerContext *PIDContext);





#endif /* INCLUDE_COURSE_DRIVE_SPEED_CS_H_*/





