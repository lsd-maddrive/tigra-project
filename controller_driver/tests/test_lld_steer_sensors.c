#include <tests.h>
#include <lld_steer_sensors.h>


/*===========================================================================*/
/* Serial driver related.                                                    */
/*===========================================================================*/

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

/*
 * @brief   Routine of steering sensors testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */

void testSteerSensorsWorking( void )
{
    lldSteerSensorsInit();

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    uint16_t adcSteerPos = 0, check = 0;


    while( true )
    {

        adcSteerPos = lldSteerPosition();
        check = lldSteerPressPower();

        chprintf((BaseSequentialStream *)&SD7, "ADC ch1: %d\n\r", adcSteerPos);
        chprintf((BaseSequentialStream *)&SD7, "ADC ch2: %d\n\r", check);
        chThdSleepMilliseconds( 100 );

    }

}
