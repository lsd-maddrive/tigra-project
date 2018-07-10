#include <tests.h>
#include <ros_proto.h>
#include <chprintf.h>

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};


void testROSDriverRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    rosInit( NORMALPRIO );

    int32_t cntr = 0;

    while ( 1 )
    {
        mainControlInfo_t   info = mainControlGetInfo();

        chprintf( (BaseSequentialStream *)&SD7, "Task speed: %d / steer: %d / mode: %d\n", info.speedTask, info.steerTask );

        chThdSleepMilliseconds( 100 );
    }
}
