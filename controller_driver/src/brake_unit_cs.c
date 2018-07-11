#include <brake_unit_cs.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

/* Constant power of cylinder return (up to the endpoint sensor) */
#define BRAKE_UNIT_RETURN_CONST_POWER   50

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/*** Typedefs ***/

/*** Variables ***/

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
        /* Set direct power */
        controlValue = pressPower;

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

