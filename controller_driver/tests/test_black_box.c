#include <tests.h>
#include <chprintf.h>
#include <black_box.h>



static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

/*
 * @brief   Routine of black box testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testBlackBoxRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    if ( blackBoxInit() < 0 )
    {
    	chprintf( (BaseSequentialStream *)&SD7, "Initialization failed\n" );
    	chSysHalt( "Test failed" );
    }

    while ( 1 )
    {
        chprintf( (BaseSequentialStream *)&SD7, "Test\n" );

        chThdSleepMilliseconds( 1000 );
    }
}
