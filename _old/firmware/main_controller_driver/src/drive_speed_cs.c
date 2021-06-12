#include <drive_speed_cs.h>
#include <brake_unit_cs.h>

static bool             isInitialized = false;


void driveSpeedCSInit( void )
{
    if ( isInitialized )
        return;

    wheelPosSensorInit();
    lldControlInit();
    brakeUnitCSInit();

    lightUnitInit();

    isInitialized = true;
}

controlValue_t driveSpeedControl ( int32_t speedReference )
{
  /* Check if all modules initialized. if not return -1 */
    if( !isInitialized )
        return 0;

    controlValue_t      motorPower          = speedReference;
    static bool         isForwardMode       = true;
    static bool         isForwardModeDes    = true;
    static bool         changingState       = false;
    static bool         setDirection        = true;

    static uint32_t     featureCntr     = 0;

    motorPower  = CLIP_VALUE( motorPower, -100, 100 );

    if ( !changingState )
    {
        if( isForwardMode && motorPower < 0 )
        {
            changingState       = true;
            isForwardModeDes    = false;

            featureCntr         = 0;
        }
        else if ( !isForwardMode && motorPower > 0 )
        {
            changingState       = true;
            isForwardModeDes    = true;

            featureCntr         = 0;
        }
        else
        {
            setDirection = isForwardMode;
            brakeUnitCSSetPower( -1 );
        }
    }
    else
    {
        motorPower = 0;
        // brakeUnitCSSetPower( 50 );

        if ( isForwardModeDes )
        {
            setDirection = true;
        }
        else
        {
            setDirection = false;
        }

        if ( featureCntr++ > 20 && !wheelPosSensorIsRotating() )
        {
            changingState = false;

            isForwardMode = isForwardModeDes;
        }
    }


    lldControlSetDrMotorPower ( motorPower );
    lldControlSetDrMotorDirection( setDirection );

    /* Lights */

    if ( !isForwardMode && motorPower != 0 )
    {
        turnLightsSetState( LIGHTS_BACK_ON );
    }
    else
    {
        turnLightsSetState( LIGHTS_BACK_OFF );
    }

    return motorPower;
}

