#include <lld_control.h>
#include <lld_brake_sensor.h>
#include <brake_unit_cs.h>

/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

#define BRAKE_UNIT_RETURN_CONST_POWER   20
#define BRAKE_UNIT_PRESS_POWER_RATE     1.0

/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

/*** Typedefs ***/

typedef struct
{
    float   p_rate;
    float   i_rate;

    float   integr_sum;

} PID_brake_ctx_t;

/*** Variables ***/
PID_brake_ctx_t   brake_pid_ctx;

/*** Functions ***/

void brakeUnitCSInit( void )
{
    /* Some initialization sequence */

    brake_pid_ctx.p_rate        = 1;
    brake_pid_ctx.i_rate        = 0;
    brake_pid_ctx.integr_sum    = 0;

    brakeSensorInit();
    lldControlInit();
}

void brakeUnitCSSetPower( int16_t pressPower )
{
    pressPower  = CLIP_VALUE( pressPower, 0, 100 );

    if ( pressPower > 0 )
    {
        int16_t current_sensor_value = 0;
        current_sensor_value = brakeSensorGetPressPower();

        int32_t error = pressPower * BRAKE_UNIT_PRESS_POWER_RATE - current_sensor_value;

        brake_pid_ctx.integr_sum += brake_pid_ctx.i_rate * error;

        int32_t control = brake_pid_ctx.p_rate * error + brake_pid_ctx.integr_sum;

        /* Set direct power */
        control = CLIP_VALUE( control, 0, 100 );

        lldControlSetBrakeDirection( true );
        lldControlSetBrakePower( control );
    }
    else
    {
        /* Set return power */
        if ( !brakeSensorIsPressed() )
        {
            /* Still not in the end - set inversed const power */
            lldControlSetBrakeDirection( false );
            lldControlSetBrakePower( BRAKE_UNIT_RETURN_CONST_POWER );
        }
        else
        {
            /* In the end - disable power */
            lldControlSetBrakePower( 0 );
        }
    }
}
