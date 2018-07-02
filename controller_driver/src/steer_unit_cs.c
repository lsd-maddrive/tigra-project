#include <steer_unit_cs.h>

#define CSErrorDeadzoneHalfwidth    2

static PIDControllerContext_t  pidCtx = {
    .kp   = 1,
    .ki   = 0,
    .kd   = 0,
    .integrLimit  = 100
};

static bool             isInitialized   = false;

/**
 * @brief   Initialize modules connected to steering control
 */
void steerUnitCSInit( void )
{
    if ( isInitialized )
        return;

    /* Some initialization sequence */
    lldSteerSensorsInit();
    lldControlInit();

    PIDControlInit( &pidCtx );

    isInitialized = true;
}

int32_t steerUnitCSSetPosition( int32_t position )
{
    if ( !isInitialized )
        return 0;

    position  = CLIP_VALUE( position, -100, 100 );

    int16_t steerPosition = lldSteerGetPositionPrc();

    int16_t error = position - steerPosition;
    
    /* Deadzone */
    if ( abs(error) < CSErrorDeadzoneHalfwidth )
    {
        pidCtx.err = 0;
    }
    else
    {
        pidCtx.err = error;
    }

    int32_t controlValue    = PIDControlResponse( &pidCtx );

    /* Is it required ? */
    if( abs(controlValue) < 10 )
    {
        controlValue = 0;
    }

    /* Set direct power */
    controlValue = CLIP_VALUE( controlValue, -40, 40 );

    lldControlSetSteerPower( controlValue );

    return controlValue;

}
