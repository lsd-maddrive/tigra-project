#include <tests.h>
#include <chprintf.h>
#include <light_unit.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};




void testLightningRoutineWorking( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lightUnitInit();

    int32_t controlValue    = 0;
    int32_t controlValDelta = 10;

    while(1)
    {
        char rcv_data = sdGet( &SD7 );
        switch ( rcv_data )
        {
            case 'e':   // Positive steer
                controlValue += controlValDelta;
                break;

            case 'r':   // Negative steer
                controlValue -= controlValDelta;
                break;

            default:
                    ;
        }

        controlValue = CLIP_VALUE( controlValue, -100, 100 );

        turnLightsSetState( controlValue );
        chprintf( (BaseSequentialStream *)&SD7, "Control Signal:%d\n\r", controlValue );
        chThdSleepMilliseconds( 100 );

    }

}
