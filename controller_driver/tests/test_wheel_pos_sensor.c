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

        chThdSleepMicroseconds( 250 );
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

        double speedIntPart;
        double speedFrctPart = modf( velocity, &speedIntPart );

        double posIntPart;
        double posFrctPart = modf( position, &posIntPart );

        chprintf( (BaseSequentialStream *)&SD7, "Vel: %d.%03d\tPos: %d.%03d\r\n" ,
                    (int)(speedIntPart), (int)(speedFrctPart * 1000),
                    (int)(posIntPart), (int)(posFrctPart * 1000) );

        chThdSleepMilliseconds( 100 );
    }
}
