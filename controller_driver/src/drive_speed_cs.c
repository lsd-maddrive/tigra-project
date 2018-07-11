#include <drive_speed_cs.h>

#define speedReferenceMaxVal    100
#define speedReferenceMinVal    -100

static bool             isInitialized = false;


void driveSpeedCSInit( void )
{
    if ( isInitialized )
        return;

    wheelPosSensorInit();
    lldControlInit();
    
    isInitialized = true;
}

bool isForward( controlValue_t motorPower )
{
    return ( motorPower >= 0);
}

controlValue_t driveSpeedControl ( int32_t speedReference )
{
  /* Check if all modules initialized. if not return -1 */
    if( !isInitialized )
        return 0;

    controlValue_t      motorPower      = speedReference;
    static bool         isForwardMode   = true;

    speedReference  = CLIP_VALUE( speedReference, speedReferenceMinVal, speedReferenceMaxVal );

    if( isForward( motorPower ) != isForwardMode )
    {
        /* wheels don't move */
        if( !wheelPosSensorIsRotating() )
        {
            isForwardMode = isForward( motorPower );
        }
        else
        {
            motorPower = 0;
        }
    }

    lldControlSetDrMotorPower ( motorPower );

    return motorPower;
}

