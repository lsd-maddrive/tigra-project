#include <tests.h>
#include <lld_sonars.h>


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
 * @brief   Routine of sonar sensors testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testSonarsRoutineWorking( void )
{
    lldSonarsInit( );

    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    uint16_t adcSonarVal = 0;


    while( true )
    {

        adcSonarVal = lldSonar7077AdcVal();
//        sdWrite(&SD7, (uint8_t*)&adcSonarVal, sizeof(adcSonarVal));
//        chprintf((BaseSequentialStream *)&SD7, "Hi\n\r");
        chprintf((BaseSequentialStream *)&SD7, "ADC: %d\n\r", adcSonarVal);

        chThdSleepMilliseconds( 300 );

    }

}
