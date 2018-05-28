#include <tests.h>
#include <chprintf.h>
#include <brake_unit_cs.h>
#include <lld_brake_sensor.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

void testBrakeUnitCSRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    brakeSensorInit();
    lldControlInit();
    brakeUnitCSInit();

    int32_t     counter = 0;
    bool        direct  = false;

    while ( 1 )
    {
        if ( direct )
            brakeUnitCSSetPower( 30 );
        else
            brakeUnitCSSetPower( 0 );

        bool                isBrakePressed  = brakeSensorIsPressed();
        brakePressPower_t   pressPower      = brakeSensorGetPressPower();

        chprintf( (BaseSequentialStream *)&SD7, "Brake: %spressed, power: %d\n",
                        isBrakePressed ? "" : "not ", pressPower );

        counter++;
        if ( counter >= 300 )
        {
            counter = 0;
            direct = !direct;
        }

        chThdSleepMilliseconds( 10 );
    }
}
