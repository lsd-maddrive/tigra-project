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


int32_t     prevControlValue    = 0;
int32_t     counter             = 0;
bool        setNull             = false;
int32_t     limitSpeed          = 0;
int32_t     pidCurrentError = 0, pidPreviousError = 0, pidInt = 0, pidDif = 0;

int32_t steerUnitCSSetPosition( int32_t position )
{
    if ( !isInitialized )
        return 0;

    position  = CLIP_VALUE( position, -100, 100 );

    int16_t steerPosition = lldSteerGetPositionPrc();

    int16_t error = position - steerPosition;
    
    /* Deadzone */
    if ( -CSErrorDeadzoneHalfwidth < pidCtx.err && pidCtx.err < CSErrorDeadzoneHalfwidth )
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

    pidPreviousError = pidCurrentError;

    lldControlSetSteerPower( controlValue );

    return controlValue;

}
