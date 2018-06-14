#include <tests.h>
#include <lld_sonars.h>


/*===========================================================================*/
/* Serial driver related.                                                    */
/*===========================================================================*/

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = (!(USART_CR1_PS | USART_CR1_M | USART_CR1_M_0)),
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

static const SerialConfig sd5cfg = {
  .speed = 9600,
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

    sdStart( &SD5, &sd5cfg );
    palSetPadMode( GPIOC, 12, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOD,  2, PAL_MODE_ALTERNATE(8) );    // RX


    uint16_t adcSonarVal = 0;
    uint16_t adcSonarVal2 = 0;
    uint8_t test_uart = 0;
    lldSonarSync();
//    USART_CR1_PS
//    sdcfg.Init.Parity =  USART_PARITY_NONE;
//    chprintf( (BaseSequentialStream *)&SD7, "testtttttt\n\r");
    while( true )
    {

//      chprintf( (BaseSequentialStream *)&SD5, "test\n\r");

      test_uart = sdGet(&SD5);
      sdWrite(&SD7, (uint8_t*)&test_uart, sizeof(test_uart));


//      chprintf( (BaseSequentialStream *)&SD7, "test: %d\n\r", test_uart);

//        msg_t msg = sdGetTimeout( &SD7, MS2ST( 10 ) );
//        adcSonarVal  = lldSonar7077AdcVal(1);
//        adcSonarVal2 = lldSonar7077AdcVal(2);
//        chprintf( (BaseSequentialStream *)&SD5, "S1: %d\n\r", adcSonarVal2);
//        sdWrite(&SD7, (uint8_t*)&adcSonarVal, sizeof(adcSonarVal));
//        chprintf((BaseSequentialStream *)&SD7, "Hi\n\r");
//        chprintf( (BaseSequentialStream *)&SD7, "S1: %d   S2: %d\n\r", adcSonarVal, adcSonarVal2);
//        chprintf( (BaseSequentialStream *)&SD7, "UART %d", msg);

        chThdSleepMilliseconds( 100 );

    }

}
