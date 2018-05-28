#include <tests.h>
#include <lld_control.h>
#include <chprintf.h>

void testDriverControlRoutine( void )
{
    lldControlInit( );

    lldControlSetSteerPower( 250 );
    lldControlSetBrakePower( 250 );
    lldControlSetDrMotorPower( 250 );

    while( true )
    {
        lldControlSetDrMotorDirection( true );
        lldControlSetBrakeDirection( true );
        lldControlSetSteerDirection( true );
        chThdSleepMilliseconds(500);
        lldControlSetDrMotorDirection( false );
        lldControlSetBrakeDirection( false );
        lldControlSetSteerDirection( false );
        chThdSleepMilliseconds(500);
    }
}

/***************************************************/

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

int16_t     steer_values_delta  = 10;
int16_t     steer_delta_sign    = 1;
int16_t     steer_value         = 0;

static THD_WORKING_AREA(waSteerThd, 128);
static THD_FUNCTION(SteerThd, arg)
{
    arg = arg;
    while ( 1 )
    {
        lldControlSetSteerDirection( steer_value > 0 );
        lldControlSetSteerPower( steer_value );

        chThdSleepMilliseconds( 100 );
    }
}

int16_t     speed_values_delta  = 10;
int16_t     speed_delta_sign    = 1;
int16_t     speed_value         = 0;

static THD_WORKING_AREA(waSpeedThd, 128);
static THD_FUNCTION(SpeedThd, arg)
{
    arg = arg;
    while ( 1 )
    {
        lldControlSetDrMotorDirection( steer_value > 0 );
        lldControlSetDrMotorPower( speed_value );

        chThdSleepMilliseconds( 100 );
    }
}

int16_t     brake_values_delta  = 10;
int16_t     brake_delta_sign    = 1;
int16_t     brake_value         = 0;

static THD_WORKING_AREA(waBrakeThd, 128);
static THD_FUNCTION(BrakeThd, arg)
{
    arg = arg;
    while ( 1 )
    {
        lldControlSetBrakeDirection( brake_value > 0 );
        lldControlSetBrakePower( brake_value );

        chThdSleepMilliseconds( 100 );
    }
}

void testDriverControlRoutineExt1( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lldControlInit();

    chThdCreateStatic( waSpeedThd, sizeof(waSpeedThd), NORMALPRIO, SpeedThd, NULL );
    chThdCreateStatic( waSteerThd, sizeof(waSteerThd), NORMALPRIO, SteerThd, NULL );
    chThdCreateStatic( waBrakeThd, sizeof(waBrakeThd), NORMALPRIO, BrakeThd, NULL );

    while ( 1 )
    {
        char rcv_data = sdGet( &SD7 );
        switch ( rcv_data )
        {
            case 'q':   // Positive steer
                steer_value += steer_values_delta * steer_delta_sign;
                if ( abs(steer_value) >= 100 )
                    steer_delta_sign *= -1;
                break;

            case 'w':   // Negative steer
                steer_value -= steer_values_delta * steer_delta_sign;
                if ( abs(steer_value) >= 100 )
                    steer_delta_sign *= -1;
                break;

            case 'a':   // Positive speed
                speed_value += speed_values_delta * speed_delta_sign;
                if ( abs(speed_value) >= 100 )
                    speed_delta_sign *= -1;
                break;

            case 's':   // Negative speed
                speed_value -= speed_values_delta * speed_delta_sign;
                if ( abs(speed_value) >= 100 )
                    speed_delta_sign *= -1;
                break;

            case 'z':   // Positive brake
                brake_value += brake_values_delta * brake_delta_sign;
                if ( abs(brake_value) >= 100 )
                    brake_delta_sign *= -1;
                break;

            case 'x':   // Negative brake
                brake_value -= brake_values_delta * brake_delta_sign;
                if ( abs(brake_value) >= 100 )
                    brake_delta_sign *= -1;
                break;

            default:
                ;
        }
        chprintf( (BaseSequentialStream *)&SD7, "Powers: %d / %d / %d\n",
                  steer_value, speed_value, brake_value );
        chThdSleepMilliseconds( 100 );
    }
}
