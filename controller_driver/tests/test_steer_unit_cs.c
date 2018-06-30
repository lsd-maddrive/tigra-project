#include <tests.h>
#include <chprintf.h>
#include <steer_unit_cs.h>


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

void testSteerUnitCSRoutine( void )
{

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    steerUnitCSInit();


    int32_t     counter = 0;
    int32_t     steerPower = 1850;
    char rcv_data = 'j';
//    char rcv_data = 0;
    while ( 1 )
    {
        int32_t currentPosSensor = lldSteerPosition();
        int32_t control      = steerUnitCSSetPower( steerPower );

        if ( ++counter >= 2 )
        {
            chprintf( (BaseSequentialStream *)&SD7, "Ref: %d, Control: %d, posSensor: %d\n\r", steerPower, control, currentPosSensor );

            counter = 0;
        }

        rcv_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        switch ( rcv_data )
        {
            case 'd':   // Positive brake
                steerPower += 10;
                break;

            case 'f':   // Negative brake
                steerPower -= 10;
                break;

            default:
                ;
        }

        steerPower = CLIP_VALUE( steerPower, 188, 3885 );
    }

    chThdSleepMilliseconds( 10 );
}
