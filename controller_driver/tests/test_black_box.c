#include <tests.h>
#include <chprintf.h>
#include <black_box.h>


static THD_WORKING_AREA(waBlinker, 128);
static THD_FUNCTION(Blinker, arg) 
{
    arg = arg;

    chRegSetThreadName("blinker");

    while ( 1 ) 
    {
        palToggleLine( LINE_LED1 );

        chThdSleepMilliseconds( 500 );
    }
}

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

    chThdCreateStatic( waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, NULL );

    if ( blackBoxInit() < 0 )
    {
    	chprintf( (BaseSequentialStream *)&SD7, "Initialization failed\n" );
    	chSysHalt( "Test failed" );
    }

    chprintf( (BaseSequentialStream *)&SD7, "Initialization done\n" );

    while ( 1 )
    {
        int res = blackBoxCardConnect();

        chprintf( (BaseSequentialStream *)&SD7, "Connection test: %s\n", res < 0 ? "Err" : "Done" );
        if ( res < 0 )
        {
            chThdSleepSeconds( 1 );
            chSysHalt( "Test failed" );
        }

        char buffer [256];
        buffer[0] = '/';
        buffer[1] = 0;
        blackBoxListFiles( (BaseSequentialStream *)&SD7, buffer );

        blackBoxCardDisconnect();

        chThdSleepMilliseconds( 1000 );
    }
}
