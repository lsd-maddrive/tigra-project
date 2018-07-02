#include <drive_speed_cs.h>

/* Configure minimum and maximum speed reference values (rpm)*/
#define speedReferenceMaxVal   30000
#define speedReferenceMinVal   0

static PIDControllerContext_t  pidCtx = {
    .kp   = 1,
    .ki   = 0.1,
    .kd   = 0,
    .integrLimit  = 100
};

static bool             isInitialized = false;


void DriveSpeedCSInit( void )
{
    if ( isInitialized )
        return
    
    wheelPosSensorInit();
    lldControlInit();
    
    PIDControlInit( &pidCtx );

    isInitialized = true;
}


controlValue_t DriveSpeedControl ( wheelVelocity_t speedReference )
{
  /* Check if all modules initialized. if not return -1 */
    if( !isInitialized )
        return 0;

    wheelVelocity_t     currentSpeed;
    controlValue_t      motorPower;

    speedReference  = CLIP_VALUE( speedReference, speedReferenceMinVal, speedReferenceMaxVal );
    currentSpeed    = wheelPosSensorGetVelocity ();

    /* Compute error and set to context */
    pidCtx.err      = speedReference - currentSpeed;

    /* float to int32_t */
    motorPower = PIDControlResponse( &pidCtx );

    lldControlSetDrMotorPower ( motorPower );

    return motorPower;
}
















