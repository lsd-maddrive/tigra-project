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
static PID_brake_ctx_t  brake_pid_ctx;
static bool             isInitialized   = false;
static int32_t          pwm_control     = 0;
/*** Functions ***/

void brakeUnitCSInit( void )
{
    if ( isInitialized )
        return;

    /* Some initialization sequence */

    brake_pid_ctx.p_rate        = 3;
    brake_pid_ctx.i_rate        = 0.07;
    brake_pid_ctx.integr_sum    = 0;

    brakeSensorInit();
    lldControlInit();

    isInitialized = true;
}

void brakeUnitCSSetPower( int16_t pressPower )
{
    if ( !isInitialized )
        return;

    pressPower  = CLIP_VALUE( pressPower, -1, 100 );

    if ( pressPower > 0 )
    {
        int16_t current_sensor_value = 0;
        current_sensor_value = brakeSensorGetPressPower();

        int32_t error = pressPower * BRAKE_UNIT_PRESS_POWER_RATE - current_sensor_value;

        brake_pid_ctx.integr_sum += error;
        brake_pid_ctx.integr_sum = CLIP_VALUE(brake_pid_ctx.integr_sum, -2000, 2000);

        pwm_control = brake_pid_ctx.p_rate * error + brake_pid_ctx.i_rate * brake_pid_ctx.integr_sum;

        /* Set direct power */
        pwm_control = CLIP_VALUE( pwm_control, -100, 100 );

//        lldControlSetBrakeDirection( true );
        lldControlSetBrakePower( pwm_control );
    }
    else if( pressPower < 0 )
    {
        /* Set return power */
        if ( !brakeSensorIsPressed() )
        {
            /* Still not in the end - set inversed const power */
//            lldControlSetBrakeDirection( false );
            lldControlSetBrakePower( -BRAKE_UNIT_RETURN_CONST_POWER );
        }
        else
        {
            /* In the end - disable power */
            lldControlSetBrakePower( 0 );
        }
    }
    else
    {
      lldControlSetBrakePower( 0 );
    }
}

int16_t brakeUnitCSGetControl ( void )
{
    return pwm_control;
}
