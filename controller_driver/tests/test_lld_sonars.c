#include <tests.h>
#include <lld_sonars.h>


#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SHARP)

static const SerialConfig sd7cfg = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = USART_CR2_LINEN,
  .cr3 = 0
};

#define portSD7                     GPIOE
#define padTX7                      8
#define padRX7                      7

void testSharpRoutineWorking( void )
{
    lldSharpInit();

    sdStart( &SD7, &sd7cfg );
    palSetPadMode( portSD7, padTX7, PAL_MODE_ALTERNATE(8) );
    palSetPadMode( portSD7, padRX7, PAL_MODE_ALTERNATE(8) );

    uint16_t sharpSunny = 0;
    uint8_t start = 0;
    bool flag = false;
    while( 1 )
    {
        start = sdGetTimeout( &SD7, TIME_IMMEDIATE );
        if( start == 1 )
          flag = true;
        else if( start == 2 )
          flag == false;

        sharpSunny = lldSharpADCval();
        if( flag )
        {
          sdWrite( &SD7, &sharpSunny, 2 );
        }

//        chprintf( (BaseSequentialStream *)&SD7, "Sharp: %d\n\r", sharpSunny );
        chThdSleepMilliseconds( 10 );
    }
}

#endif

#if (MAIN_PROGRAM_ROUTINE == PROGRAM_ROUTINE_TEST_LL_SONAR)
/*
 * @brief   Routine of sonar sensors testing
 * @brief   Printing values (uint16_t) into Terminal
 * @note    The routine has internal infinite loop
 * @note    SD7 is used for testing (PE7, PE8)
 */
void testSonarsRoutineWorking( void )
{
    lldSonarsInit( );
    uint16_t blueSonar = 0, brownSonar = 0;
    while( true )
    {
      palToggleLine( LINE_LED2 );
      blueSonar  =  getSonarValU7cm();
      brownSonar =  getSonarValU4cm();

      chprintf( (BaseSequentialStream *)&SD7, "Br: %d Bl: %d\n\r", brownSonar, blueSonar );

      chThdSleepMilliseconds( 100 );

    }

}
#endif
