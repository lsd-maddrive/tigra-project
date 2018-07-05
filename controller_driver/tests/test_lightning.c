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
    int32_t counter         = 0;
    chprintf( (BaseSequentialStream *)&SD7, "TEST\n\r" );

    while(1)
    {

        if( ++counter >= 10 )
        {
            chprintf( (BaseSequentialStream *)&SD7, "Control Signal:%d\n\r", controlValue );
            counter = 0;
        }
        char rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'q':   // Positive steer
                controlValue += controlValDelta;
                break;

            case 'w':   // Negative steer
                controlValue -= controlValDelta;
                break;

            default:
                    ;
        }

        controlValue = CLIP_VALUE( controlValue, -100, 100 );

        /*      Light Unit      */
        if( controlValue <= -20 )
        {
            turnLightsSetState( LIGHTS_TURN_LEFT );
        }
        else if( controlValue >= 20 )
        {
            turnLightsSetState( LIGHTS_TURN_RIGHT );
        }
        else
        {
          turnLightsSetState( LIGHTS_TURN_OFF );
        }

        chThdSleepMilliseconds( 10 );

    }

}
