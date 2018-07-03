#include <tests.h>
#include <chprintf.h>
#include <lld_control.h>
#include <lld_steer_sensors.h>
#include <steer_unit_cs.h>


static const SerialConfig sdcfg = {
    .speed = 115200,
    .cr1 = 0, .cr2 = 0, .cr3 = 0
  };


/**
 * @brief   Steer power testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 * @note    steerIsEnabled() gives 1s delay
 */
void testSteerPoweredStatus( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    /* Some initialization sequence */
    lldControlInit();
    lldSteerSensorsInit();


    static bool statusESC       = true;
    uint16_t counter            = 0;

    statusESC = steerIsEnabled();

    while( 1 )
    {
        counter += 1;
        if( counter == 50 )
        {
          chprintf( (BaseSequentialStream *)&SD7, "Status of ESC: %s\n\r", statusESC ? "true" : "false" );

          counter = 0;
        }

        chThdSleepMilliseconds( 10 );
    }

}
