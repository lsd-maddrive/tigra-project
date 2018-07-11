#include <brake_unit_cs.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

/* Constant power of cylinder return (up to the endpoint sensor) */
#define BRAKE_UNIT_RETURN_CONST_POWER   50
/* Rate for conversion from reference to comparison with sensor output */
#define BRAKE_UNIT_PRESS_POWER_RATE     1.0f

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/*** Typedefs ***/

/*** Variables ***/

static PIDControllerContext_t  pidCtx = {
    .kp   = 3,
    .ki   = 0.07,
    .kd   = 0,
    .integrLimit  = 200
};

static bool             isInitialized   = false;
/*** Functions ***/

void brakeUnitCSInit( void )
{
    if ( isInitialized )
        return;

    /* Some initialization sequence */
    brakeSensorInit();
    lldControlInit();

    lightUnitInit();

    PIDControlInit( &pidCtx );

    isInitialized = true;
}

controlValue_t brakeUnitCSSetPower( int16_t pressPower )
{
    if ( !isInitialized )
        return 0;

    controlValue_t      controlValue;

    pressPower  = CLIP_VALUE( pressPower, -1, 100 );

    if ( pressPower > 0 )
    {
        int16_t current_sensor_value = 0;
        current_sensor_value = brakeSensorGetPressPower();

        pidCtx.err = pressPower * BRAKE_UNIT_PRESS_POWER_RATE - current_sensor_value;

        controlValue = PIDControlResponse( &pidCtx );

        /* Set direct power */
        controlValue = CLIP_VALUE( controlValue, -100, 100 );

        turnLightsSetState( LIGHTS_BRAKE_ON );
    }
    else if( pressPower < 0 )
    {
        /* Set return power */
        if ( !brakeSensorIsPressed() )
        {
            /* Still not in the end - set inverted const power */
            controlValue = -BRAKE_UNIT_RETURN_CONST_POWER;
        }
        else
        {
            /* In the end - disable power */
            controlValue = 0;
        }

        turnLightsSetState( LIGHTS_BRAKE_OFF );
    }
    else
    {
        /* Just disable the system */
        controlValue = 0;

        turnLightsSetState( LIGHTS_BRAKE_OFF );
    }

    lldControlSetBrakePower( controlValue );

    return controlValue;
}

