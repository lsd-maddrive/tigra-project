#include <tests.h>
#include <lld_control.h>
#include <chprintf.h>

void testDriverControlRoutine( void )
{
    lldControlInit( );

    lldControlSetSteerPower( 250 );
    lldControlSetBrakePower( 75 );
    lldControlSetDrMotorPower( 250 );
    int8_t test_val = 75;
    while( true )
    {

       lldControlSetBrakePower( test_val );
       test_val *= -1;

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
int8_t      brake_value         = 0;

static THD_WORKING_AREA(waBrakeThd, 128);
static THD_FUNCTION(BrakeThd, arg)
{
    arg = arg;
    while ( 1 )
    {

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
                steer_value = CLIP_VALUE( steer_value, -100, 100 );
                break;

            case 'w':   // Negative steer
                steer_value -= steer_values_delta * steer_delta_sign;
                steer_value = CLIP_VALUE( steer_value, -100, 100 );
                break;

            case 'a':   // Positive speed
                speed_value += speed_values_delta * speed_delta_sign;
                speed_value = CLIP_VALUE( speed_value, -100, 100 );
                break;

            case 's':   // Negative speed
                speed_value -= speed_values_delta * speed_delta_sign;
                speed_value = CLIP_VALUE( speed_value, -100, 100 );
                break;

            case 'z':   // Positive brake
                brake_value += brake_values_delta * brake_delta_sign;
                brake_value = CLIP_VALUE( brake_value, -100, 100 );
                break;

            case 'x':   // Negative brake
                brake_value -= brake_values_delta * brake_delta_sign;
                brake_value = CLIP_VALUE( brake_value, -100, 100 );
                break;

            default:
                ;
        }

        chprintf( (BaseSequentialStream *)&SD7, "Powers:\tSteer (%d)\n\tSpeed(%d)\n\tBrake(%d)\n",
                  steer_value, speed_value, brake_value );
        chThdSleepMilliseconds( 100 );
    }
}
