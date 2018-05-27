#include <course_drive_speed_cs.h>
#include <lld_wheel_pos_sensor.h>
#include <lld_control.h>

static bool isInitialized = false;

/* Configure minimum and maximum speed reference values (rpm)*/
#define speedReferenceMaxVal   30000
#define speedReferenceMinVal   0

/* Configuration - PID controller parameters*/
static controllerParams PIDParmsCfg = {
                                        .kp = 1,
                                        .ki = 0.1,
                                        .kd = 2
};

/* Switching context - input parameter to controller function */
static controllerContext PIDContext = {
                                        .err    = 0,
                                        .params = &PIDParmsCfg
};


/*
 *@brief         Low level drivers initialization
 *@note          First time function call sets flag "isInitialized"
 *               which protects of multiple initialization
 */
void CourseDriveSpeedCSInit( void )
{
   if (!isInitialized )
   {
       wheelPosSensorInit();
       lldControlInit();
   }
   isInitialized = true;
}


/*
 *  @brief       Control system law realization. PID controller *
 *  @params[in]  Structure, contains current error, PID controller parameters
 *  @params[out] Controller output [0;100] %
 *  @note        Access parameters like this:
 *                                            PIDContext->params->kp
 *                                            PIDContext->err
 */
uint8_t PIDController (controllerContext *PIDContext)
{
  uint8_t PIDOut = 0;
  PIDOut = PIDContext->params->kp * PIDContext->err;

  if ( PIDOut > 100 )
    PIDOut = 100;
  if ( PIDOut < 0 )
    PIDOut = 0;

  return PIDOut;
}


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
uint8_t CourseDriveSpeedControl (wheelVelocity_t speedReference )
{
  /* Check if all modules initialized. if not return -1 */
    if( !isInitialized )
      return -1;

  /* Speed reference saturation */
    if ( speedReference > speedReferenceMaxVal )
      speedReference = speedReferenceMaxVal;
    if ( speedReference < speedReferenceMinVal )
      speedReference = speedReferenceMinVal;

    wheelVelocity_t currentSpeed    =   wheelPosSensorGetVelocity ();
    PIDContext.err  =   speedReference - currentSpeed;
    uint8_t         lldMotorPower   =   PIDController(&PIDContext);
    bool lldDrMotorDirection = true;
    lldControlSetDrMotorDirection ( lldDrMotorDirection );
    lldControlSetDrMotorPower ( lldMotorPower );
    return lldMotorPower;
}
















