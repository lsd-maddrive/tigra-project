#ifndef INCLUDE_COURSE_DRIVE_SPEED_CS_H_
#define INCLUDE_COURSE_DRIVE_SPEED_CS_H_

#include <common.h>
#include <lld_wheel_pos_sensor.h>

/***Hardware description***/


/*** Variables ***/

//typedef float       speedError_t;
//typedef float       speedReference_t;
typedef float         coeffPIDController;

typedef struct {

  coeffPIDController    kp;
  coeffPIDController    ki;
  coeffPIDController    kd;

}controllerParams;

typedef struct {

    wheelVelocity_t       err;
    controllerParams      *params;

}controllerContext;


/*** Prototypes ***/



/*
 * @brief
 */
void CourseDriveSpeedCSInit( void );

/*
 * @brief
 */
void CourseDriveSpeedControl (wheelVelocity_t speedReference );

/*
 *  @brief
 *  @params [in]
 *  @params [out]
 *  @note
 *
 */
uint8_t PIDController (controllerContext *PIDContext);


#endif /* INCLUDE_COURSE_DRIVE_SPEED_CS_H_*/
