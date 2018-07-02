#include <steer_unit_cs.h>

static bool             isInitialized   = false;

static PIDControllerContext_t  pidCtx = {
    .kp   = 1,
    .ki   = 0,
    .kd   = 0,
    .integrLimit  = 100
};

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

    int16_t steerPosition = lldSteerGetPosition();

    pidCtx.err = position - steerPosition;

    int32_t controlValue    = PIDControlResponse( &pidCtx );

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
