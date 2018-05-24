#include <tests.h>
#include <chprintf.h>
#include <lld_wheel_pos_sensor.h>

#ifdef TEST_WHEEL_POS_SENSOR_SIMULATED

static THD_WORKING_AREA(waBtnThd, 128);
static THD_FUNCTION(PosSensorOutThd, arg)
{
    arg = arg;

    /* Required for direct control */
    palSetPadMode( GPIOF, 14, PAL_MODE_OUTPUT_PUSHPULL );

    while ( 1 )
    {
        palTogglePad( GPIOF, 14 );
        chThdSleepMilliseconds( 50 );

    }
}

static void simulation_init ( void )
{

    chThdCreateStatic( waBtnThd, sizeof(waBtnThd), NORMALPRIO, PosSensorOutThd, NULL );
}

#endif  //TEST_WHEEL_POS_SENSOR_SIMULATED

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

/*
 * @brief   Routine of break sensor testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
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
    uint16_t serial_test = 0;
    while ( 1 )
    {

        wheelVelocity_t velocity   =   wheelPosSensorGetVelocity ();
        wheelPosition_t position   =   wheelPosSensorGetPosition ();

        /*chprintf( (BaseSequentialStream *)&SD7, "%s %d\r\n %s %d\r\n" , "pos:",
                  position, "vel:", velocity);
                  */
        sendTestInformation ();
        chThdSleepMilliseconds( 1 );


    }
}
