#include <steer_unit_cs.h>

#define CSErrorDeadzoneHalfwidth        2

#define steerCheckPercPower             5
#define steerCheckNumberInstruction     100


static PIDControllerContext_t  pidCtx = {
    .kp   = 1.5,
    .ki   = 0.05,
    .kd   = 0,
    .integrLimit  = 100
};

static bool             isInitialized   = false;


/**
 * @brief   Check ESC condition
 * @return  true  - ESC enable, everything - OK
 *          false - ESC is disabled or another bad situation
 * @note    WARNING! There is a delay (1 s) inside the function
 * @note    do not call it inside the loop, use it before Control System processing
 * @note    this function works only for determined phase order, be careful
 */
bool steerIsEnabled( void )
{
     lldControlSetSteerPower( steerCheckPercPower );

     static int32_t sensorValPercSum    = 0;
     uint16_t counter                   = 0;


     while( counter <= steerCheckNumberInstruction )
     {
         counter += 1;

         sensorValPercSum += lldSteerGetCurrentPrc();

         chThdSleepMilliseconds( 10 );
     }

     sensorValPercSum /= steerCheckNumberInstruction;

     return ( sensorValPercSum > steerCheckPercPower );
}

/**
 * @brief   Initialize modules connected to steering control
 */
void steerUnitInit( void )
{
    /* Some initialization sequence */
    lldSteerSensorsInit();
    lldControlInit();

    lightUnitInit();
}

/**
 * @brief   Initialize modules connected to steering control
 */
void steerUnitCSInit( void )
{
    if ( isInitialized )
        return;

    steerUnitInit();

    PIDControlInit( &pidCtx );

    isInitialized = true;
}


/**
 * @brief   Control system of steering implementation
 * @param   position        - reference position
 * @return  controlValue    - control signal value [-100 100] ( in percent )
 */
int32_t steerUnitCSSetPosition( int32_t position )
{
    if ( !isInitialized )
        return 0;

    position  = CLIP_VALUE( position, -100, 100 );

    int16_t steerPosition = lldSteerGetPositionPrc();

    int16_t error = position - steerPosition;
    
    /* Dead zone for (p) error */
    if ( abs(error) < CSErrorDeadzoneHalfwidth )
    {
        pidCtx.err = 0;
    }
    else
    {
        pidCtx.err = error;
    }

    int32_t controlValue    = PIDControlResponse( &pidCtx );

    /*  roughly reset integral */
    if( pidCtx.err == 0 )
    {
        pidCtx.integrSum = 0;
    }

    /* Dead zone for control signal */
    if( abs(controlValue) <= 15 )
    {
        controlValue = 0;
    }

    /*  Set direct power */
    controlValue = CLIP_VALUE( controlValue, -80, 80 );

    /*      Light Unit      */
    if( position <= -20 )
    {
        turnLightsSetState( LIGHTS_TURN_LEFT );
    }
    else if( position >= 20 )
    {
        turnLightsSetState( LIGHTS_TURN_RIGHT );
    }
    else
    {
        turnLightsSetState( LIGHTS_TURN_OFF );
    }

    lldControlSetSteerPower( controlValue );

    return controlValue;

}
