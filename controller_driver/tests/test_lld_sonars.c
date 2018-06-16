#include <tests.h>
#include <lld_sonars.h>


/*===========================================================================*/
/* Serial driver related.                                                    */
/*===========================================================================*/

static const SerialConfig sd7cfg = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

static const SerialConfig sd5cfg = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

/*** Variable for sonar   */
uint16_t brownSonarVal = 0;
uint8_t buf5Son[4];

static THD_WORKING_AREA(waGetSonarValU5Thd, 1024);
static THD_FUNCTION(GetSonarValU5Thd, arg)
{
    arg = arg;
    uint8_t firstR = 0;
    while( 1 )
    {
        firstR = sdGet( &SD5 );

        if( firstR == 'R' )
        {
            sdRead( &SD5, buf5Son, 3 );
            buf5Son[3] = 0;
            brownSonarVal = strtoul( buf5Son, NULL, 0 );  // convert bufSon into string, after this srt convert into long
        }
    }
}

uint8_t buf7Son[4];
uint16_t greenSonarVal = 0;
static THD_WORKING_AREA(waGetSonarValU7Thd, 1024);
static THD_FUNCTION(GetSonarValU7Thd, arg)
{
    arg = arg;
    uint8_t firstR = 0;
    while( 1 )
    {
        firstR = sdGet( &SD7 );

        if( firstR == 'R' )
        {
            sdRead( &SD7, buf7Son, 3 );
            buf7Son[3] = 0;
            greenSonarVal = strtoul( buf7Son, NULL, 0 );  // convert bufSon into string, after this srt convert into long
        }
    }
}


/*
 * @brief   Routine of sonar sensors testing
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testSonarsRoutineWorking( void )
{
    lldSonarsInit( );

    sdStart( &SD7, &sd7cfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    sdStart( &SD5, &sd5cfg );
    palSetPadMode( GPIOC, 12, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOD,  2, PAL_MODE_ALTERNATE(8) );    // RX

    chThdCreateStatic( waGetSonarValU5Thd, sizeof(waGetSonarValU5Thd), NORMALPRIO, GetSonarValU5Thd, NULL ); // brown
    chThdCreateStatic( waGetSonarValU7Thd, sizeof(waGetSonarValU7Thd), NORMALPRIO, GetSonarValU7Thd, NULL ); // green



    while( true )
    {
      chprintf( (BaseSequentialStream *)&SD7, "Br: %d Bl: %d\n\r", brownSonarVal, greenSonarVal );
//      chprintf( (BaseSequentialStream *)&SD7, "%d\n\r", brownSonarVal );
//      test_uart = sdGet(&SD5);
//      sdWrite(&SD7, (uint8_t*)&test_uart, sizeof(test_uart));
//      sdWrite( &SD7, (uint8_t*)&greenSonarVal, sizeof(greenSonarVal) );
//      sdWrite( &SD7, (uint8_t*)&brownSonarVal, sizeof(brownSonarVal) );
//      chprintf( (BaseSequentialStream *)&SD7, "%d\n\r", adcSonarVal );
//      if(count == 100)
//      {
//        palToggleLine( LINE_LED1 );
//        count = 0;
//      }

//      chprintf( (BaseSequentialStream *)&SD7, "test: %d\n\r", test_uart);

//        msg_t msg = sdGetTimeout( &SD7, MS2ST( 10 ) );
//        adcSonarVal  = lldSonar7077AdcVal(1);
//        adcSonarVal2 = lldSonar7077AdcVal(2);
//        chprintf( (BaseSequentialStream *)&SD5, "S1: %d\n\r", adcSonarVal2);
//        sdWrite(&SD7, (uint8_t*)&adcSonarVal, sizeof(adcSonarVal));
//        chprintf((BaseSequentialStream *)&SD7, "Hi\n\r");
//        chprintf( (BaseSequentialStream *)&SD7, "S1: %d   S2: %d\n\r", adcSonarVal, adcSonarVal2);
//        chprintf( (BaseSequentialStream *)&SD7, "UART %d", msg);

        chThdSleepMilliseconds( 10 );

    }

}
