#include <course_drive_speed_cs.h>
#include <lld_wheel_pos_sensor.h>
#include <lld_control.h>

static bool isInitialized = false;

/* Configure minimum and maximum speed reference values (rpm)*/
#define speedReferenceMaxVal   30000
#define speedReferenceMinVal   0

static controllerParams PIDParmsCfg = { .kp = 1, .ki = 0.1, .kd = 2    };
static controllerContext PIDContext = { .err = 0, .params = &PIDParmsCfg };

/*
 *@brief
 */

void CourseDriveSpeedCSInit( void )
{
    wheelPosSensorInit();
    lldControlInit();
    isInitialized = true;
}

uint8_t PIDController (controllerContext *PIDContext)
{
//  uint8_t PIDOut = PIDContext->params->kp * PIDContext->err;

  uint8_t PIDOut = 0;
  PIDContext->params->kp = 0;
  PIDContext->params->ki = 0;
  PIDContext->params->kd = 0;

  return PIDOut;
}

/*
 * @brief
 * @params [in]  speedReference
 */
void CourseDriveSpeedControl (wheelVelocity_t speedReference )
{
  /* Check if all modules initialized
   * if not?????
   *
   */

  /* Speed reference saturation */
    if ( speedReference > speedReferenceMaxVal )
      speedReference = speedReferenceMaxVal;
    if ( speedReference < speedReferenceMinVal )
      speedReference = speedReferenceMinVal;

    wheelVelocity_t currentSpeed    =   wheelPosSensorGetVelocity ();
    PIDContext.err  =   speedReference - currentSpeed;
    uint8_t         lldMotorPower   =   PIDController(&PIDContext);
    lldControlSetDrMotorPower ( lldMotorPower );
}
















