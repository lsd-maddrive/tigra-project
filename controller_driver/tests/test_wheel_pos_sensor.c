#include <tests.h>
#include <chprintf.h>
#include <lld_wheel_pos_sensor.h>

#ifdef TEST_WHEEL_POS_SENSOR_SIMULATED

static THD_WORKING_AREA(waPosSensorThd, 128);
static THD_FUNCTION(PosSensorOutThd, arg)
{
    arg = arg;

    /* Required for direct control */
    palSetPadMode( GPIOF, 14, PAL_MODE_OUTPUT_PUSHPULL );

    while ( 1 )
    {
        palTogglePad( GPIOF, 14 );

        //chThdSleepMilliseconds(2);
        //chThdSleepSeconds(40);
        chThdSleepMicroseconds(250);

    }
}

static void simulation_init ( void )
{

    chThdCreateStatic( waPosSensorThd, sizeof(waPosSensorThd), NORMALPRIO, PosSensorOutThd, NULL );
}

#endif  //TEST_WHEEL_POS_SENSOR_SIMULATED

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

void testWheelPosSensorRoutine( void )
{
#ifdef TEST_WHEEL_POS_SENSOR_SIMULATED
    simulation_init();
    palClearLine(LINE_LED1);
#endif

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    wheelPosSensorInit();

    while ( 1 )
    {

        wheelVelocity_t velocity   =   wheelPosSensorGetVelocity ();
        wheelPosition_t position   =   wheelPosSensorGetPosition ();

        /* Send to serial port position and velocity
         * fixed point representation (3 decimals)*/
        chprintf( (BaseSequentialStream *)&SD7, "%s %d\r\n %s %d\r\n" , "pos:",
                  (int) (position*1000), "vel:", (int) (velocity*1000) );

        //sendTestInformation ();
        chThdSleepMilliseconds( 100 );


    }
}
